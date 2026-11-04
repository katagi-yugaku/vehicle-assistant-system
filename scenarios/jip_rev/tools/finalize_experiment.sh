#!/usr/bin/env bash
# 全タスク終了後に走る集計ジョブ(sbatch --dependency=afterany:<配列ジョブ>)。
#   finalize_experiment.sh RUN_ROOT [--dry-run-mail]
#
# 1. status.txt を数え、予定数と成功数を比べる
# 2. aggregate_experiment.py で results/<実験ID>/ に summary.csv・図などを作る
# 3. 生データ(runs/<実験ID>、凍結コードを含む)を archive/<実験ID>.tar.zst(なければ .tar.gz)に固め、
#    tar -t で中身を確認し、sha256 を付けて NAS(EVAC_ARCHIVE_DEST)へコピーして照合する
# 4. 結果をメールで送る(件名の末尾: SUCCESS / INCOMPLETE / FAILED)
# 生データ(runs/<実験ID>)は消さない。消すのは NAS の照合と結果の確認が済んでから、手で行う。
set -uo pipefail

RUN_ROOT="$(cd -- "${1:?RUN_ROOT is required}" && pwd -P)"; shift
DRY_RUN_MAIL=0
for a in "$@"; do [[ "$a" == --dry-run-mail ]] && DRY_RUN_MAIL=1; done

TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"     # 凍結した tools
JIP_REV="${EVAC_JIP_REV:-$(cd -- "${RUN_ROOT}/../.." && pwd -P)}"   # scenarios/jip_rev
PY="${EVAC_PYTHON:-python3}"
EXP_ID="$(basename "${RUN_ROOT}")"
RESULTS="${JIP_REV}/results/${EXP_ID}"
ARCHIVE_DIR="${EVAC_ARCHIVE_LOCAL:-${JIP_REV}/archive}"
LOG="${RUN_ROOT}/finalize.log"
mkdir -p "${RESULTS}" "${ARCHIVE_DIR}"
exec > >(tee -a "${LOG}") 2>&1

# NAS の保存先(どちらか):
#   EVAC_ARCHIVE_DEST=/mnt/debussy/evac-sim/archive       (計算ノードからマウントされているパス)
#   EVAC_ARCHIVE_DEST=debussy:/path/to/evac-sim/archive   (rsync over ssh。鍵認証が必要)
# ~/.config/evac-sim/archive.env に「export EVAC_ARCHIVE_DEST=...」と書いておけば読み込む。
[[ -f "${HOME}/.config/evac-sim/archive.env" ]] && source "${HOME}/.config/evac-sim/archive.env"
ARCHIVE_DEST="${EVAC_ARCHIVE_DEST:-}"

echo "== finalize ${EXP_ID} on $(hostname) at $(date '+%F %T')"
read -r PLANNED SUBMITTED < <("${PY}" -c "import json;m=json.load(open('${RUN_ROOT}/manifest.json'));print(m['planned_runs'],m['submitted_at'])")
SUCCESS="$(find "${RUN_ROOT}" -name status.txt -path '*/r[0-9]*' -exec grep -l '^status=SUCCESS$' {} + 2>/dev/null | wc -l | tr -d ' ')"
FAILED="$(find "${RUN_ROOT}" -name status.txt -path '*/r[0-9]*' -exec grep -L '^status=SUCCESS$' {} + 2>/dev/null | wc -l | tr -d ' ')"
MISSING=$(( PLANNED - SUCCESS - FAILED ))
echo "planned=${PLANNED} success=${SUCCESS} failed=${FAILED} missing=${MISSING}"

"${PY}" "${TOOLS}/aggregate_experiment.py" "${RUN_ROOT}" --out "${RESULTS}"
echo "aggregate exit=$?"

if [[ "${SUCCESS}" -eq "${PLANNED}" && -s "${RESULTS}/summary.csv" ]]; then OUTCOME=SUCCESS
elif [[ "${SUCCESS}" -gt 0 && -s "${RESULTS}/summary.csv" ]]; then OUTCOME=INCOMPLETE
else OUTCOME=FAILED; fi

# ---- 生データの保存 ----
# tar が --zstd に対応している場合だけ zstd を使う(CentOS 7 の tar 1.26 は非対応)
if command -v zstd >/dev/null 2>&1 && tar --zstd -cf /dev/null --files-from /dev/null >/dev/null 2>&1; then ARCHIVE="${ARCHIVE_DIR}/${EXP_ID}.tar.zst"; COMP=(--zstd)
else ARCHIVE="${ARCHIVE_DIR}/${EXP_ID}.tar.gz"; COMP=(-z); fi
ARCHIVE_NOTE="archive FAILED (tar)"
if tar -C "$(dirname "${RUN_ROOT}")" "${COMP[@]}" -cf "${ARCHIVE}.part" "${EXP_ID}" \
   && tar "${COMP[@]}" -tf "${ARCHIVE}.part" > /dev/null; then
  mv -f "${ARCHIVE}.part" "${ARCHIVE}"
  (cd "${ARCHIVE_DIR}" && sha256sum "$(basename "${ARCHIVE}")" > "$(basename "${ARCHIVE}").sha256")
  ARCHIVE_NOTE="local ${ARCHIVE} ($(du -h "${ARCHIVE}" | cut -f1))"
  if [[ -z "${ARCHIVE_DEST}" ]]; then
    ARCHIVE_NOTE="${ARCHIVE_NOTE}; NAS: EVAC_ARCHIVE_DEST 未設定のためコピーせず"
  elif [[ "${ARCHIVE_DEST}" == *:* ]]; then
    if rsync -a "${ARCHIVE}" "${ARCHIVE}.sha256" "${ARCHIVE_DEST}/" \
       && [[ -z "$(rsync -a --checksum --dry-run --itemize-changes "${ARCHIVE}" "${ARCHIVE_DEST}/")" ]]; then
      ARCHIVE_NOTE="${ARCHIVE_NOTE}; NAS ${ARCHIVE_DEST}/$(basename "${ARCHIVE}") (checksum OK)"
    else
      ARCHIVE_NOTE="${ARCHIVE_NOTE}; NAS copy FAILED (${ARCHIVE_DEST})"; fi
  else
    if mkdir -p "${ARCHIVE_DEST}" && cp "${ARCHIVE}" "${ARCHIVE}.sha256" "${ARCHIVE_DEST}/" \
       && (cd "${ARCHIVE_DEST}" && sha256sum -c --quiet "$(basename "${ARCHIVE}").sha256"); then
      ARCHIVE_NOTE="${ARCHIVE_NOTE}; NAS ${ARCHIVE_DEST}/$(basename "${ARCHIVE}") (sha256 OK)"
    else
      ARCHIVE_NOTE="${ARCHIVE_NOTE}; NAS copy FAILED (${ARCHIVE_DEST})"; fi
  fi
fi
echo "archive: ${ARCHIVE_NOTE}"

# ---- メール ----
BODY="${RUN_ROOT}/mail_body.txt"
{
  echo "実験: ${EXP_ID}"
  echo "結果: ${OUTCOME}(成功 ${SUCCESS} / 予定 ${PLANNED}、失敗 ${FAILED}、未実行 ${MISSING})"
  echo "投入: ${SUBMITTED} / 集計: $(date '+%FT%T') / ホスト: $(hostname)"
  echo "集計結果: ${RESULTS}"
  echo "生データ: ${ARCHIVE_NOTE}"
  echo "元の生データ: ${RUN_ROOT}(消していません)"
  echo
  cat "${RESULTS}/decision.md" 2>/dev/null || echo "(decision.md がありません)"
  if [[ "${FAILED}" -gt 0 ]]; then
    echo; echo "== 失敗した実行(最大 10 件)の run.err 末尾 =="
    find "${RUN_ROOT}" -name status.txt -path '*/r[0-9]*' -exec grep -L '^status=SUCCESS$' {} + 2>/dev/null | head -10 |
    while read -r st; do d="$(dirname "${st}")"; echo "--- ${d#${RUN_ROOT}/}"; grep -v '^Warning' "${d}/run.err" | tail -n 15; done
  fi
} > "${BODY}"
cp "${LOG}" "${RESULTS}/finalize.log" 2>/dev/null || true

MAIL=(--subject "[evac-sim] ${EXP_ID} ${OUTCOME}" --body-file "${BODY}"
      --attach "${RESULTS}"/figs/*.png "${RESULTS}"/figs/*.pdf "${RESULTS}/summary.csv" "${RESULTS}/paired.csv" "${RESULTS}/decision.md")
[[ ${DRY_RUN_MAIL} -eq 1 ]] && MAIL+=(--dry-run)
"${PY}" "${TOOLS}/notify_mail.py" "${MAIL[@]}"
echo "== finalize done: ${OUTCOME}"
[[ "${OUTCOME}" == SUCCESS ]]
