#!/usr/bin/env bash
#SBATCH --job-name=vehassist
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=02:00:00
#SBATCH --output=logs/%x_%j.out
#SBATCH --error=logs/%x_%j.err

set -euo pipefail

# 受け取り：
# 引数1 = scenarioID
# 引数2 = v2v_rate
# 引数3 = run_id 任意
SCENARIO_ID="${1:?scenarioID is required}"
V2V_RATE="${2:?v2v_rate is required}"
RUN_ID="${3:-0}"

# early_rate は固定
EARLY_RATE="1.0"

# JIP 用 config
CONFIG="scenarios/JIP/configs/config_scenario_${SCENARIO_ID}.toml"

# config が存在するか確認
if [[ ! -f "${CONFIG}" ]]; then
  echo "ERROR: config file not found: ${CONFIG}" >&2
  exit 1
fi

mkdir -p logs

echo "=== Job Info ==="
echo "host      : $(hostname)"
echo "jobid     : ${SLURM_JOB_ID:-N/A}"
echo "scenario  : ${SCENARIO_ID}"
echo "early_rate: ${EARLY_RATE}"
echo "v2v_rate  : ${V2V_RATE}"
echo "run_id    : ${RUN_ID}"
echo "config    : ${CONFIG}"
echo "start     : $(date)"

export RUN_ID

python3 -W ignore::UserWarning -m scenarios.JIP.1-1-4.simulation.runner_simulator \
  --nogui "${CONFIG}" "${EARLY_RATE}" "${V2V_RATE}"

echo "end       : $(date)"