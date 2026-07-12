#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  ./batch_jobs.sh SCENARIO_FROM SCENARIO_TO TARGET_SCENARIO_DIR

Examples:
  ./batch_jobs.sh 1 36 1-1-2
  ./batch_jobs.sh 24 25 1-1-4
  ./batch_jobs.sh 70 73 1-1-4-v2vonly
USAGE
}

if (( $# != 3 )); then
  usage >&2
  exit 2
fi

SCENARIO_FROM="$1"
SCENARIO_TO="$2"
TARGET_SCENARIO_DIR="$3"

SCRIPT_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
  pwd -P
)"
cd "${SCRIPT_DIR}"

if [[ ! "${SCENARIO_FROM}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: SCENARIO_FROM must be an integer: ${SCENARIO_FROM}" >&2
  exit 2
fi

if [[ ! "${SCENARIO_TO}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: SCENARIO_TO must be an integer: ${SCENARIO_TO}" >&2
  exit 2
fi

if (( SCENARIO_FROM > SCENARIO_TO )); then
  echo "ERROR: SCENARIO_FROM must be <= SCENARIO_TO" >&2
  exit 2
fi

if [[ ! "${TARGET_SCENARIO_DIR}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "ERROR: Invalid TARGET_SCENARIO_DIR: ${TARGET_SCENARIO_DIR}" >&2
  exit 2
fi

TARGET_RUNNER_PATH="${SCRIPT_DIR}/scenarios/JIP/${TARGET_SCENARIO_DIR}/simulation/runner_simulator.py"

if [[ ! -f "${TARGET_RUNNER_PATH}" ]]; then
  echo "ERROR: runner_simulator.py was not found:" >&2
  echo "  ${TARGET_RUNNER_PATH}" >&2
  exit 2
fi

SLURM_SCRIPT="${SCRIPT_DIR}/slurm_vehicle_assistant.sh"

if [[ ! -f "${SLURM_SCRIPT}" ]]; then
  echo "ERROR: Slurm script was not found: ${SLURM_SCRIPT}" >&2
  exit 2
fi

# 各条件の実行回数
n=50

# early_rateは固定
early_rate="1.0"

# v2v_rate
v2v_capable_vehicle_rate_list=(
  "1.0"
)

# 使用を許可するノード
# 1ジョブにつき、この中のいずれか1台を割り当てる。
ALIVE_NODES="paganini,elgar,chopin"

mkdir -p logs

echo "Submitting jobs..."
echo "project root    : ${SCRIPT_DIR}"
echo "target directory: ${TARGET_SCENARIO_DIR}"
echo "target runner   : ${TARGET_RUNNER_PATH}"
echo "scenario range  : ${SCENARIO_FROM}..${SCENARIO_TO}"
echo "n               : ${n}"
echo "early_rate      : ${early_rate}"
echo "v2v_list        : (${v2v_capable_vehicle_rate_list[*]})"
echo "allowed nodes   : ${ALIVE_NODES}"
echo "nodes per job   : 1"

for ((
  scenarioID = SCENARIO_FROM;
  scenarioID <= SCENARIO_TO;
  scenarioID++
)); do
  config_path="${SCRIPT_DIR}/scenarios/JIP/configs/config_scenario_${scenarioID}.toml"

  if [[ ! -f "${config_path}" ]]; then
    echo "ERROR: config file was not found: ${config_path}" >&2
    exit 2
  fi

  for v2v_rate in "${v2v_capable_vehicle_rate_list[@]}"; do
    for ((run_id = 1; run_id <= n; run_id++)); do
      job_name="va_s${scenarioID}_e${early_rate}_v${v2v_rate}_r${run_id}"

    sbatch \
      --partition=ubuntu \
      --nodes=1 \
      --ntasks=1 \
      --job-name="${job_name}" \
      "${SLURM_SCRIPT}" \
      "${scenarioID}" \
      "${v2v_rate}" \
      "${run_id}" \
      "${TARGET_SCENARIO_DIR}"
    done
  done
done

echo "Done."