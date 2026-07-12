#!/usr/bin/env bash
#SBATCH --job-name=vehassist
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=02:00:00
#SBATCH --output=logs/%x_%j.out
#SBATCH --error=logs/%x_%j.err

set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  slurm_vehicle_assistant.sh \
    SCENARIO_ID \
    V2V_RATE \
    RUN_ID \
    TARGET_SCENARIO_DIR

Example:
  slurm_vehicle_assistant.sh 24 1.0 1 1-1-4-v2vonly
USAGE
}

if (( $# < 4 )); then
  usage >&2
  exit 2
fi

# 引数
SCENARIO_ID="${1:?scenarioID is required}"
V2V_RATE="${2:?v2v_rate is required}"
RUN_ID="${3:?run_id is required}"
TARGET_SCENARIO_DIR="${4:?target scenario directory is required}"

# early_rateは固定
EARLY_RATE="1.0"

if [[ ! "${SCENARIO_ID}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: SCENARIO_ID must be an integer: ${SCENARIO_ID}" >&2
  exit 2
fi

if [[ ! "${RUN_ID}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: RUN_ID must be an integer: ${RUN_ID}" >&2
  exit 2
fi

if [[ ! "${TARGET_SCENARIO_DIR}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "ERROR: Invalid TARGET_SCENARIO_DIR: ${TARGET_SCENARIO_DIR}" >&2
  exit 2
fi

# sbatchを実行したディレクトリをプロジェクトルートとして使用する。
PROJECT_ROOT="${SLURM_SUBMIT_DIR:-$(pwd)}"
cd "${PROJECT_ROOT}"

CONFIG="scenarios/JIP/configs/config_scenario_${SCENARIO_ID}.toml"

RUNNER_PATH="scenarios/JIP/${TARGET_SCENARIO_DIR}/simulation/runner_simulator.py"

RUNNER_MODULE="scenarios.JIP.${TARGET_SCENARIO_DIR}.simulation.runner_simulator"

if [[ ! -f "${CONFIG}" ]]; then
  echo "ERROR: config file not found: ${CONFIG}" >&2
  exit 1
fi

if [[ ! -f "${RUNNER_PATH}" ]]; then
  echo "ERROR: runner_simulator.py not found: ${RUNNER_PATH}" >&2
  exit 1
fi

mkdir -p logs

echo "=== Job Info ==="
echo "host            : $(hostname)"
echo "jobid           : ${SLURM_JOB_ID:-N/A}"
echo "project_root    : ${PROJECT_ROOT}"
echo "scenario        : ${SCENARIO_ID}"
echo "early_rate      : ${EARLY_RATE}"
echo "v2v_rate        : ${V2V_RATE}"
echo "run_id          : ${RUN_ID}"
echo "config          : ${CONFIG}"
echo "target_directory: ${TARGET_SCENARIO_DIR}"
echo "runner_path     : ${RUNNER_PATH}"
echo "runner_module   : ${RUNNER_MODULE}"
echo "start           : $(date '+%Y-%m-%d %H:%M:%S')"

export RUN_ID

python3 \
  -W ignore::UserWarning \
  -m "${RUNNER_MODULE}" \
  --nogui \
  "${CONFIG}" \
  "${EARLY_RATE}" \
  "${V2V_RATE}"

echo "end             : $(date '+%Y-%m-%d %H:%M:%S')"