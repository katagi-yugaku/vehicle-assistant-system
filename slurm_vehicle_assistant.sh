#!/usr/bin/env bash
#SBATCH --job-name=vehassist
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=02:00:00
#SBATCH --output=logs/%x_%j.out
#SBATCH --error=logs/%x_%j.err

set -euo pipefail

# 受け取り：引数1=scenarioID, 引数2=early_rate, 引数3=v2v_rate, 引数4=run_id(任意)
SCENARIO_ID="${1:?scenarioID is required}"
EARLY_RATE="${2:?early_rate is required}"
V2V_RATE="${3:?v2v_rate is required}"
RUN_ID="${4:-0}"

CONFIG="scenarios/pervehicle/configs/config_scenario_${SCENARIO_ID}.toml"

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

# ここで必要なら環境ロード（例）
# module load python/3.x
# source ~/venv/bin/activate
# conda activate yourenv

# 任意：run_id を seed に使えるなら環境変数で渡す（runner側で読む実装なら）
export RUN_ID

python3 -m scenarios.pervehicle.map_one.simulation.runner_simulator \
  --nogui \
  "${CONFIG}" \
  "${EARLY_RATE}" \
  "${V2V_RATE}"

echo "end       : $(date)"
