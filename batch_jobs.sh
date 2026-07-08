#!/usr/bin/env bash
set -euo pipefail

# 引数：from to
SCENARIO_FROM="${1:?scenarioID_from is required}"
SCENARIO_TO="${2:?scenarioID_to is required}"

# 実験設定
n=50

# early_rate は固定
early_rate=1.0

# v2v_rate のみ sweep
v2v_capable_vehicle_rate_list=(1.0)

# mount が正常なノードだけ使う
ALIVE_NODES="paganini,elgar,chopin"

mkdir -p logs

# 範囲チェック
if (( SCENARIO_FROM > SCENARIO_TO )); then
  echo "ERROR: scenarioID_from must be <= scenarioID_to"
  exit 1
fi

echo "Submitting jobs..."
echo "scenario range: ${SCENARIO_FROM}..${SCENARIO_TO}"
echo "n=${n}"
echo "early_rate=${early_rate}"
echo "v2v_list=(${v2v_capable_vehicle_rate_list[*]})"
echo "nodes=${ALIVE_NODES}"

for (( scenarioID=SCENARIO_FROM; scenarioID<=SCENARIO_TO; scenarioID++ )); do
  for v2v_rate in "${v2v_capable_vehicle_rate_list[@]}"; do
    for run_id in $(seq 1 "${n}"); do
      sbatch \
        --partition=ubuntu \
        --job-name="va_s${scenarioID}_e${early_rate}_v${v2v_rate}_r${run_id}" \
        ./slurm_vehicle_assistant.sh \
        "${scenarioID}" \
        "${v2v_rate}" \
        "${run_id}"
    done
  done
done

echo "Done."