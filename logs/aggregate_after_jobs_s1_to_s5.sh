#!/usr/bin/env bash
set -euo pipefail

cd "/home/katagi/vehicle-assistant-system"

echo "[INFO] aggregate job started"
echo "[INFO] working directory: $(pwd)"
echo "[INFO] expected branch: auto/aggregate-s1-to-s5-20260512-182615"

# -----------------------------------------
# branch 確認
# aggregate job 側では checkout しない
# 同じ作業ディレクトリで別 branch に切り替えられていたら止める
# -----------------------------------------
if [[ "1" == "1" ]]; then
  current_branch="$(git rev-parse --abbrev-ref HEAD)"

  if [[ "${current_branch}" != "auto/aggregate-s1-to-s5-20260512-182615" ]]; then
    echo "ERROR: aggregate job 実行時の branch が想定と異なります"
    echo "expected: auto/aggregate-s1-to-s5-20260512-182615"
    echo "actual: ${current_branch}"
    echo "同じ作業ディレクトリで別ブランチに切り替えた可能性があります"
    echo "安全のため、集計・commit・push を中止します"
    exit 1
  fi
fi

# -----------------------------------------
# 古い同一 scenario の集計結果を削除
# -----------------------------------------
echo "[INFO] remove old aggregated outputs for target scenarios"

for (( scenarioID=1; scenarioID<=5; scenarioID++ )); do
  rm -rf "/home/katagi/vehicle-assistant-system/aggregated/scenario${scenarioID}"
done

# -----------------------------------------
# 集計を実行
# -----------------------------------------
echo "[INFO] run aggregation"
python3 "/home/katagi/vehicle-assistant-system/aggregate_simulation_logs.py" "/home/katagi/vehicle-assistant-system/logs" --output-dir "/home/katagi/vehicle-assistant-system/aggregated"

echo "[INFO] aggregation finished"
echo "[INFO] aggregated output dir: /home/katagi/vehicle-assistant-system/aggregated"

# -----------------------------------------
# 集計後の生成物を push
# -----------------------------------------
if [[ "1" == "1" ]]; then
  echo "[INFO] git add aggregated results"

  # output.json / CDF 図などの集計結果を追加
  # .gitignore に引っかかる可能性があるため -f を付ける
  git add -f "/home/katagi/vehicle-assistant-system/aggregated"

  if git diff --cached --quiet; then
    echo "[INFO] commit 対象の集計結果はありません"
  else
    git commit -m "Add aggregated simulation results"
  fi

  echo "[INFO] push branch: origin/auto/aggregate-s1-to-s5-20260512-182615"
  git push -u "origin" "auto/aggregate-s1-to-s5-20260512-182615"
else
  echo "[INFO] AUTO_GIT_PUSH=0 のため git add / commit / push はスキップします"
fi

echo "[INFO] aggregate job finished"
