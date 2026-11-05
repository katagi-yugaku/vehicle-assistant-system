#!/usr/bin/env bash
# 1 タスク(= 1 回のシミュレーション)を実行する。
#   Slurm  : sbatch --array=1-N ... run_task.sh RUN_ROOT   (SLURM_ARRAY_TASK_ID を使う)
#   ローカル: run_task.sh RUN_ROOT TASK_ID
# 凍結したコード(RUN_ROOT/frozen/code)だけを使うので、投入後にブランチやファイルを
# 変えても実行中のジョブには影響しない。
# 結果: <run_dir>/{run.out, run.err, raw.json, tripinfo.xml, status.txt}
set -uo pipefail

RUN_ROOT="${1:?RUN_ROOT is required}"
TASK_ID="${2:-${SLURM_ARRAY_TASK_ID:-}}"
[[ -n "${TASK_ID}" ]] || { echo "ERROR: TASK_ID (arg 2) or SLURM_ARRAY_TASK_ID is required" >&2; exit 2; }
PY="${EVAC_PYTHON:-python3}"

LINE="$(awk -F'\t' -v id="${TASK_ID}" 'NR > 1 && $1 == id' "${RUN_ROOT}/tasks.tsv")"
[[ -n "${LINE}" ]] || { echo "ERROR: task ${TASK_ID} not found in ${RUN_ROOT}/tasks.tsv" >&2; exit 2; }
IFS=$'\t' read -r _ LAYOUT COND VARIANT SEED CONFIG RUN_DIR EARLY_RATE V2V_RATE <<< "${LINE%$'\r'}"
# 早期避難率と通信可能な車両の割合は版ごとに tasks.tsv に入っている(古い tasks.tsv なら manifest の値)
if [[ -z "${EARLY_RATE:-}" || -z "${V2V_RATE:-}" ]]; then
  read -r EARLY_RATE V2V_RATE < <("${PY}" -c "import json;m=json.load(open('${RUN_ROOT}/manifest.json'));print(m['early_rate'],m['v2v_rate'])")
fi

mkdir -p "${RUN_DIR}"
cd "${RUN_DIR}"   # tripinfo.xml は作業ディレクトリに書かれるので、実行ごとに分ける

START_EPOCH="$(date +%s)"; START="$(date '+%Y-%m-%dT%H:%M:%S')"
echo "task=${TASK_ID} layout=${LAYOUT} condition=${COND} variant=${VARIANT} seed=${SEED} early_rate=${EARLY_RATE} v2v_rate=${V2V_RATE} host=$(hostname) start=${START}"

export PYTHONPATH="${RUN_ROOT}/frozen/code"
export PYTHONHASHSEED=0
"${PY}" -W ignore::UserWarning \
  -m "scenarios.JIP.${LAYOUT}.simulation.runner_simulator" \
  --nogui "${CONFIG}" "${EARLY_RATE}" "${V2V_RATE}" \
  --seed "${SEED}" --raw-output "${RUN_DIR}/raw.json" \
  > "${RUN_DIR}/run.out" 2> "${RUN_DIR}/run.err"
RC=$?

STATUS=SUCCESS; REASON=""
if [[ ${RC} -ne 0 ]]; then STATUS=FAILED; REASON="exit code ${RC}"
elif [[ ! -s "${RUN_DIR}/raw.json" ]]; then STATUS=FAILED; REASON="raw.json missing"
elif ! grep -q "\[RUN_CONFIG\].*seed=${SEED} " "${RUN_DIR}/run.out"; then STATUS=FAILED; REASON="seed not confirmed in run.out"
fi

{
  echo "status=${STATUS}"; echo "exit_code=${RC}"; echo "reason=${REASON}"
  echo "task_id=${TASK_ID}"; echo "layout=${LAYOUT}"; echo "condition=${COND}"
  echo "variant=${VARIANT}"; echo "seed=${SEED}"; echo "early_rate=${EARLY_RATE}"; echo "v2v_rate=${V2V_RATE}"
  echo "host=$(hostname)"
  echo "slurm_job=${SLURM_ARRAY_JOB_ID:-}_${SLURM_ARRAY_TASK_ID:-}"
  echo "start=${START}"; echo "end=$(date '+%Y-%m-%dT%H:%M:%S')"
  echo "elapsed_s=$(( $(date +%s) - START_EPOCH ))"
} > "${RUN_DIR}/status.txt.tmp" && mv -f "${RUN_DIR}/status.txt.tmp" "${RUN_DIR}/status.txt"

echo "status=${STATUS} rc=${RC} elapsed=$(( $(date +%s) - START_EPOCH ))s ${REASON}"
[[ "${STATUS}" == SUCCESS ]]
