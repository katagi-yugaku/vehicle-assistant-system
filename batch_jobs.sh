#!/usr/bin/env bash
set -euo pipefail

# 引数：from to
SCENARIO_FROM="${1:?scenarioID_from is required}"
SCENARIO_TO="${2:?scenarioID_to is required}"

# 実験設定
n=10
early_rate_list=(0.1 0.5 0.9)
v2v_capable_vehicle_rate_list=(0.0 0.5 1.0)

base_early_rate=0.5
base_v2v_capable_vehicle_rate=1.0

mkdir -p logs

# 範囲チェック（簡易）
if (( SCENARIO_FROM > SCENARIO_TO )); then
  echo "ERROR: scenarioID_from must be <= scenarioID_to"
  exit 1
fi

echo "Submitting jobs..."
echo "scenario range: ${SCENARIO_FROM}..${SCENARIO_TO}"
echo "n=${n}"
echo "early_rate_list=(${early_rate_list[*]})"
echo "v2v_list=(${v2v_capable_vehicle_rate_list[*]})"
echo "base_early_rate=${base_early_rate}"
echo "base_v2v_rate=${base_v2v_capable_vehicle_rate}"

for (( scenarioID=SCENARIO_FROM; scenarioID<=SCENARIO_TO; scenarioID++ )); do

  # (1) early_rate sweep（v2vは固定）
  for early_rate in "${early_rate_list[@]}"; do
    for run_id in $(seq 1 "${n}"); do
      sbatch \
        --job-name="va_s${scenarioID}_e${early_rate}_v${base_v2v_capable_vehicle_rate}_r${run_id}" \
        ./slurm_vehicle_assistant.sh \
        "${scenarioID}" \
        "${early_rate}" \
        "${base_v2v_capable_vehicle_rate}" \
        "${run_id}"
    done
  done

  # (2) v2v sweep（earlyは固定）
  for v2v_rate in "${v2v_capable_vehicle_rate_list[@]}"; do
    for run_id in $(seq 1 "${n}"); do
      sbatch \
        --job-name="va_s${scenarioID}_e${base_early_rate}_v${v2v_rate}_r${run_id}" \
        ./slurm_vehicle_assistant.sh \
        "${scenarioID}" \
        "${base_early_rate}" \
        "${v2v_rate}" \
        "${run_id}"
    done
  done

done

echo "Done."
