#!/usr/bin/env bash
# e2e テストの投入と判定(計算機の scenarios/jip_rev で実行)。手順と合格基準は revision/E2E_TEST_PLAN.md。
#
#   tools/e2e_run.sh preflight              # 静的チェック + 全実験定義の --print-only(投入しない)
#   tools/e2e_run.sh submit  [--local]      # E2E-layouts / E2E-comm / 再現性用の再実行(E2E-layouts-rep)を投入
#   tools/e2e_run.sh check                  # 終わった 3 実験を判定(runs/ の最新の ID を使う)
#
# 環境変数: EVAC_REF(既定 origin/jip/revision-26S015f)、EVAC_PYTHON
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "${HERE}/.." && pwd -P)"
cd "${ROOT}"
PY="${EVAC_PYTHON:-python3}"
REF="${EVAC_REF:-origin/jip/revision-26S015f}"
CMD="${1:-}"; shift || true
BACKEND=(); [[ "${1:-}" == --local ]] && BACKEND=(--backend local --max-parallel "${EVAC_MAX_PARALLEL:-4}")

latest() { ls -1d runs/*_"$1"_* 2>/dev/null | sort | tail -1; }

case "${CMD}" in
  preflight)
    "${PY}" tools/e2e_preflight.py
    for spec in experiments/E2.toml experiments/E2_none.toml experiments/E3.toml \
                experiments/e2e/E2E_layouts.toml experiments/e2e/E2E_comm.toml; do
      echo "== print-only ${spec}"
      out="$(tools/run_experiment.sh "${spec}" --ref "${REF}" --print-only --name "pf$(date +%s%N | tail -c 6)" 2>&1)" || { echo "${out}"; exit 1; }
      echo "${out}" | grep -E '^\[experiment\]'
      rid="$(echo "${out}" | sed -n 's/^\[experiment\] \([^ ]*\).*/\1/p')"
      [[ -n "${rid}" && -d "runs/${rid}" ]] && rm -rf "runs/${rid}"   # print-only の作業ディレクトリは消す
    done
    ;;
  submit)
    tools/run_experiment.sh experiments/e2e/E2E_layouts.toml --ref "${REF}" "${BACKEND[@]}"
    tools/run_experiment.sh experiments/e2e/E2E_comm.toml    --ref "${REF}" "${BACKEND[@]}"
    # 再現性(D1):同じコード・同じシードで 4 回だけもう一度
    tools/run_experiment.sh experiments/e2e/E2E_layouts.toml --ref "${REF}" --seeds 1-1 --name E2E-layouts-rep "${BACKEND[@]}"
    ;;
  check)
    A="$(latest E2E-layouts)"; B="$(latest E2E-comm)"; C="$(latest E2E-layouts-rep)"
    rc=0
    [[ -n "${A}" ]] && { "${PY}" tools/e2e_check.py "${A}" ${C:+--compare-to "${C}"} || rc=1; } || { echo "E2E-layouts がありません"; rc=1; }
    [[ -n "${B}" ]] && { "${PY}" tools/e2e_check.py "${B}" || rc=1; } || { echo "E2E-comm がありません"; rc=1; }
    echo "== e2e $([[ ${rc} -eq 0 ]] && echo PASS || echo FAIL)"
    exit ${rc}
    ;;
  *) sed -n '2,8p' "$0"; exit 2 ;;
esac
