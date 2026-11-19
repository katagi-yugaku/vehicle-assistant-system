#!/usr/bin/env bash
# 計算機での一連の流れ:最新の改訂ブランチを取り込む → 実験定義を確認 → 投入。
# 終わると集計ジョブが結果(図・表・伝達と判断の分析)をメールで送る。
#
#   cd ~/vehicle-assistant-system/scenarios/jip_rev
#   tools/server_run.sh experiments/E2s_9ms.toml                 # 投入
#   tools/server_run.sh experiments/E2s_9ms.toml --print-only    # 投入せずに確認だけ
#   (2 つ目以降の引数はそのまま run_experiment.py に渡す。例: --seeds 1-1 --name E2s-smoke)
#
# - 取り込みは「git fetch + jip_rev だけを作業ツリーに展開」。main のブランチは切り替えない(pull と同じ効果)。
# - 実行するコードは投入時に origin/<ブランチ> のコミットから凍結する(run_experiment.py --ref)。
# - メールの宛先は ~/.config/evac-mail/gmail.ini の to。環境変数 EVAC_MAIL_TO があればそちらを使う
#   (~/.config/evac-sim/mail.env に「export EVAC_MAIL_TO=...」と書いておけば読み込む)。
set -euo pipefail

main() {
  local spec="${1:?usage: tools/server_run.sh experiments/<E>.toml [run_experiment の引数...]}"; shift
  local branch="${EVAC_BRANCH:-jip/revision-26S015f}"
  local here repo
  here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"       # scenarios/jip_rev
  repo="$(cd "${here}" && git rev-parse --show-toplevel 2>/dev/null || (cd "${here}/../.." && pwd -P))"   # 古い git は -C 非対応
  [[ -f "${HOME}/.config/evac-sim/mail.env" ]] && source "${HOME}/.config/evac-sim/mail.env"

  echo "== 1. 取り込み (origin/${branch})"
  (cd "${repo}" && git fetch origin && git archive "origin/${branch}" scenarios/jip_rev | tar -x)
  echo "   commit: $(cd "${repo}" && git log -1 --oneline "origin/${branch}")"

  cd "${here}"
  echo "== 2. 実験定義の確認"
  "${EVAC_PYTHON:-python3}" tools/e2e_preflight.py "${spec}"

  echo "== 3. 投入"
  echo "   メールの宛先: ${EVAC_MAIL_TO:-(gmail.ini の to)}"
  tools/run_experiment.sh "${spec}" --ref "origin/${branch}" "$@"
}

main "$@"; exit $?
