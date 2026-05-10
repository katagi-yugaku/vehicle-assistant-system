#!/usr/bin/env bash
set -euo pipefail

# =========================================
# 引数：from to
# =========================================
SCENARIO_FROM="${1:?scenarioID_from is required}"
SCENARIO_TO="${2:?scenarioID_to is required}"

# =========================================
# Git 設定
# =========================================
AUTO_GIT_PUSH="${AUTO_GIT_PUSH:-1}"
GIT_REMOTE="${GIT_REMOTE:-origin}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
AGGREGATE_SCRIPT_PATH="${SCRIPT_DIR}/aggregate_simulation_logs.py"

GIT_BRANCH_NAME="${GIT_BRANCH_NAME:-auto/aggregate-s${SCENARIO_FROM}-to-s${SCENARIO_TO}-$(date +%Y%m%d-%H%M%S)}"

# =========================================
# 実験設定
# =========================================
n=50
early_rate_list=(0.1 0.5 0.9)

# v2v は固定
base_v2v_capable_vehicle_rate=1.0

LOG_DIR="${SCRIPT_DIR}/logs"
AGGREGATED_OUTPUT_DIR="${SCRIPT_DIR}/aggregated"

mkdir -p "${LOG_DIR}"
mkdir -p "${AGGREGATED_OUTPUT_DIR}"

# =========================================
# 範囲チェック
# =========================================
if (( SCENARIO_FROM > SCENARIO_TO )); then
  echo "ERROR: scenarioID_from must be <= scenarioID_to"
  exit 1
fi

# =========================================
# 事前チェック
# =========================================
if [[ ! -f "${AGGREGATE_SCRIPT_PATH}" ]]; then
  echo "ERROR: aggregate_simulation_logs.py が同一階層に存在しません: ${AGGREGATE_SCRIPT_PATH}"
  exit 1
fi

if ! git -C "${SCRIPT_DIR}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "ERROR: このスクリプトの配置場所は Git リポジトリ内ではありません: ${SCRIPT_DIR}"
  exit 1
fi

if [[ "${AUTO_GIT_PUSH}" == "1" ]]; then
  if ! git -C "${SCRIPT_DIR}" remote get-url "${GIT_REMOTE}" >/dev/null 2>&1; then
    echo "ERROR: Git remote '${GIT_REMOTE}' が存在しません"
    exit 1
  fi

  echo "[INFO] Git branch を作成します: ${GIT_BRANCH_NAME}"
  git -C "${SCRIPT_DIR}" checkout -b "${GIT_BRANCH_NAME}"
else
  echo "[INFO] AUTO_GIT_PUSH=0 のため Git branch 作成・push はスキップします"
fi

# =========================================
# simulation job 投入
# =========================================
echo "Submitting simulation jobs..."
echo "scenario range: ${SCENARIO_FROM}..${SCENARIO_TO}"
echo "n=${n}"
echo "early_rate_list=(${early_rate_list[*]})"
echo "fixed_v2v_rate=${base_v2v_capable_vehicle_rate}"
echo "git_branch=${GIT_BRANCH_NAME}"

job_ids=()

for (( scenarioID=SCENARIO_FROM; scenarioID<=SCENARIO_TO; scenarioID++ )); do
  for early_rate in "${early_rate_list[@]}"; do
    for run_id in $(seq 1 "${n}"); do

      job_name="va_s${scenarioID}_e${early_rate}_v${base_v2v_capable_vehicle_rate}_r${run_id}"

      job_id=$(
        sbatch \
          --parsable \
          --job-name="${job_name}" \
          ./slurm_vehicle_assistant.sh \
          "${scenarioID}" \
          "${early_rate}" \
          "${base_v2v_capable_vehicle_rate}" \
          "${run_id}"
      )

      # Slurm が jobid;cluster の形式で返す環境に対応
      job_id="${job_id%%;*}"

      job_ids+=("${job_id}")

      echo "[INFO] submitted: ${job_name} -> job_id=${job_id}"

    done
  done
done

echo "[INFO] simulation job count: ${#job_ids[@]}"

if (( ${#job_ids[@]} == 0 )); then
  echo "ERROR: simulation job が1件も投入されていません"
  exit 1
fi

# =========================================
# 集計 job を依存関係付きで投入
# =========================================
dependency_job_ids=$(IFS=:; echo "${job_ids[*]}")

aggregate_job_name="aggregate_s${SCENARIO_FROM}_to_s${SCENARIO_TO}"
aggregate_job_script="${LOG_DIR}/aggregate_after_jobs_s${SCENARIO_FROM}_to_s${SCENARIO_TO}.sh"

cat > "${aggregate_job_script}" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "${SCRIPT_DIR}"

echo "[INFO] aggregate job started"
echo "[INFO] working directory: \$(pwd)"
echo "[INFO] branch: ${GIT_BRANCH_NAME}"

# -----------------------------------------
# 対象ブランチへ移動
# -----------------------------------------
if [[ "${AUTO_GIT_PUSH}" == "1" ]]; then
  echo "[INFO] checkout branch: ${GIT_BRANCH_NAME}"
  git checkout "${GIT_BRANCH_NAME}"
fi

# -----------------------------------------
# 集計を実行
# -----------------------------------------
echo "[INFO] run aggregation"
python3 "${AGGREGATE_SCRIPT_PATH}" "${LOG_DIR}" --output-dir "${AGGREGATED_OUTPUT_DIR}"

echo "[INFO] aggregation finished"
echo "[INFO] aggregated output dir: ${AGGREGATED_OUTPUT_DIR}"

# -----------------------------------------
# 集計後の生成物を push
# -----------------------------------------
if [[ "${AUTO_GIT_PUSH}" == "1" ]]; then
  echo "[INFO] git add scripts and aggregated results"

  git add "${AGGREGATE_SCRIPT_PATH}" "${SCRIPT_PATH}"

  # output.json / CDF 図などの集計結果を追加
  # .gitignore に引っかかる可能性があるため -f を付ける
  git add -f "${AGGREGATED_OUTPUT_DIR}"

  if git diff --cached --quiet; then
    echo "[INFO] commit 対象の変更はありません"
  else
    git commit -m "Add aggregated simulation results"
  fi

  echo "[INFO] push branch: ${GIT_REMOTE}/${GIT_BRANCH_NAME}"
  git push -u "${GIT_REMOTE}" "${GIT_BRANCH_NAME}"
else
  echo "[INFO] AUTO_GIT_PUSH=0 のため git add / commit / push はスキップします"
fi

echo "[INFO] aggregate job finished"
EOF

chmod +x "${aggregate_job_script}"

echo "[INFO] aggregation job を投入します"
echo "[INFO] dependency=afterany:${dependency_job_ids}"
echo "[INFO] aggregate job script=${aggregate_job_script}"

aggregate_job_id=$(
  sbatch \
    --parsable \
    --job-name="${aggregate_job_name}" \
    --dependency="afterany:${dependency_job_ids}" \
    --chdir="${SCRIPT_DIR}" \
    --output="${LOG_DIR}/%x_%j.out" \
    --error="${LOG_DIR}/%x_%j.err" \
    "${aggregate_job_script}"
)

aggregate_job_id="${aggregate_job_id%%;*}"

echo "[INFO] aggregation job submitted: ${aggregate_job_name} -> job_id=${aggregate_job_id}"
echo "All simulation jobs were submitted."
echo "Aggregation will start automatically after all simulation jobs finish."
echo "Aggregated results will be committed and pushed after aggregation."