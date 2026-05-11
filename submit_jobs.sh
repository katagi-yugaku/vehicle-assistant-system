#!/usr/bin/env bash
set -euo pipefail

# =========================================
# 引数：from to
# =========================================
SCENARIO_FROM="${1:?scenarioID_from is required}"
SCENARIO_TO="${2:?scenarioID_to is required}"

# =========================================
# Git 設定
# =========================================
AUTO_GIT_PUSH="${AUTO_GIT_PUSH:-1}"
GIT_REMOTE="${GIT_REMOTE:-origin}"

# simulation job が失敗しても集計するなら afterany
# 全 simulation job が成功した場合だけ集計するなら:
#   AGGREGATE_DEPENDENCY_TYPE=afterok ./submit_jobs.sh 1 3
AGGREGATE_DEPENDENCY_TYPE="${AGGREGATE_DEPENDENCY_TYPE:-afterany}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

SCRIPT_PATH="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
AGGREGATE_SCRIPT_PATH="${SCRIPT_DIR}/aggregate_simulation_logs.py"
SBATCH_SCRIPT_PATH="${SCRIPT_DIR}/slurm_vehicle_assistant.sh"

GIT_BRANCH_NAME="${GIT_BRANCH_NAME:-auto/aggregate-s${SCENARIO_FROM}-to-s${SCENARIO_TO}-$(date +%Y%m%d-%H%M%S)}"

# =========================================
# 実験設定
# =========================================
n=50

# scenarioID 1 の比較条件
scenario1_system_early_rate_list=(0.1 0.5 0.9)
scenario1_nosystem_early_rate=0.5

# scenarioID 2 以降のチューニング条件
tuning_early_rate=1.0

# システムあり / なし
system_v2v_rate=1.0
nosystem_v2v_rate=0.0


# =========================================
# 基本チェック
# =========================================
if ! [[ "${SCENARIO_FROM}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: scenarioID_from must be an integer: ${SCENARIO_FROM}"
  exit 1
fi

if ! [[ "${SCENARIO_TO}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: scenarioID_to must be an integer: ${SCENARIO_TO}"
  exit 1
fi

if (( SCENARIO_FROM > SCENARIO_TO )); then
  echo "ERROR: scenarioID_from must be <= scenarioID_to"
  exit 1
fi

case "${AGGREGATE_DEPENDENCY_TYPE}" in
  afterany|afterok)
    ;;
  *)
    echo "ERROR: AGGREGATE_DEPENDENCY_TYPE must be afterany or afterok: ${AGGREGATE_DEPENDENCY_TYPE}"
    exit 1
    ;;
esac

if ! command -v git >/dev/null 2>&1; then
  echo "ERROR: git command not found"
  exit 1
fi

if ! command -v sbatch >/dev/null 2>&1; then
  echo "ERROR: sbatch command not found"
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 command not found"
  exit 1
fi

if [[ ! -f "${AGGREGATE_SCRIPT_PATH}" ]]; then
  echo "ERROR: aggregate_simulation_logs.py が同一階層に存在しません: ${AGGREGATE_SCRIPT_PATH}"
  exit 1
fi

if [[ ! -f "${SBATCH_SCRIPT_PATH}" ]]; then
  echo "ERROR: slurm_vehicle_assistant.sh が同一階層に存在しません: ${SBATCH_SCRIPT_PATH}"
  exit 1
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "ERROR: このスクリプトの配置場所は Git リポジトリ内ではありません: ${SCRIPT_DIR}"
  exit 1
fi

# =========================================
# job 作成前に branch 作成・switch
# =========================================
if [[ "${AUTO_GIT_PUSH}" == "1" ]]; then
  if ! git remote | grep -qx "${GIT_REMOTE}"; then
    echo "ERROR: Git remote '${GIT_REMOTE}' が存在しません"
    exit 1
  fi

  echo "[INFO] Git branch を作成して switch します: ${GIT_BRANCH_NAME}"
  git checkout -b "${GIT_BRANCH_NAME}"

  echo "[INFO] submit / aggregate scripts を先に commit 対象として固定します"
  git add "${SCRIPT_PATH}" "${AGGREGATE_SCRIPT_PATH}"

  if git diff --cached --quiet; then
    echo "[INFO] script に commit 対象の変更はありません"
  else
    git commit -m "Add aggregation automation scripts"
  fi
else
  echo "[INFO] AUTO_GIT_PUSH=0 のため Git branch 作成・push はスキップします"
fi

# =========================================
# branch switch 後に生成物ディレクトリを作成
# =========================================
LOG_DIR="${SCRIPT_DIR}/logs"
AGGREGATED_OUTPUT_DIR="${SCRIPT_DIR}/aggregated"

mkdir -p "${LOG_DIR}"
mkdir -p "${AGGREGATED_OUTPUT_DIR}"

# =========================================
# simulation job 投入
# =========================================
echo "Submitting simulation jobs..."
echo "scenario range: ${SCENARIO_FROM}..${SCENARIO_TO}"
echo "n=${n}"
echo "scenario1_system_early_rate_list=(${scenario1_system_early_rate_list[*]})"
echo "scenario1_nosystem_early_rate=${scenario1_nosystem_early_rate}"
echo "tuning_early_rate=${tuning_early_rate}"
echo "system_v2v_rate=${system_v2v_rate}"
echo "nosystem_v2v_rate=${nosystem_v2v_rate}"
echo "git_branch=${GIT_BRANCH_NAME}"
echo "aggregate_dependency_type=${AGGREGATE_DEPENDENCY_TYPE}"


# =========================================
# simulation job 投入
# =========================================
job_ids=()

submit_simulation_job() {
  local scenarioID="$1"
  local early_rate="$2"
  local v2v_rate="$3"
  local run_id="$4"
  local condition_label="$5"

  local job_name="va_s${scenarioID}_${condition_label}_e${early_rate}_v${v2v_rate}_r${run_id}"

  local job_id
  job_id=$(
    sbatch \
      --parsable \
      --job-name="${job_name}" \
      --chdir="${SCRIPT_DIR}" \
      --output="${LOG_DIR}/%x_%j.out" \
      --error="${LOG_DIR}/%x_%j.err" \
      "${SBATCH_SCRIPT_PATH}" \
      "${scenarioID}" \
      "${early_rate}" \
      "${v2v_rate}" \
      "${run_id}"
  )

  # Slurm が jobid;cluster の形式で返す環境に対応
  job_id="${job_id%%;*}"

  job_ids+=("${job_id}")

  echo "[INFO] submitted: ${job_name} -> job_id=${job_id}"
}

for (( scenarioID=SCENARIO_FROM; scenarioID<=SCENARIO_TO; scenarioID++ )); do

  if (( scenarioID == 1 )); then
    # -----------------------------------------
    # scenarioID 1:
    # システムあり early_rate = 0.1, 0.5, 0.9
    # -----------------------------------------
    for early_rate in "${scenario1_system_early_rate_list[@]}"; do
      for run_id in $(seq 1 "${n}"); do
        submit_simulation_job \
          "${scenarioID}" \
          "${early_rate}" \
          "${system_v2v_rate}" \
          "${run_id}" \
          "system"
      done
    done

    # -----------------------------------------
    # scenarioID 1:
    # システムなし early_rate = 0.5
    # -----------------------------------------
    for run_id in $(seq 1 "${n}"); do
      submit_simulation_job \
        "${scenarioID}" \
        "${scenario1_nosystem_early_rate}" \
        "${nosystem_v2v_rate}" \
        "${run_id}" \
        "nosystem"
    done

  else
    # -----------------------------------------
    # scenarioID 2 以降:
    # early_rate = 1.0，システムありのみ
    # -----------------------------------------
    for run_id in $(seq 1 "${n}"); do
      submit_simulation_job \
        "${scenarioID}" \
        "${tuning_early_rate}" \
        "${system_v2v_rate}" \
        "${run_id}" \
        "system"
    done

  fi

done

echo "[INFO] simulation job count: ${#job_ids[@]}"

if (( ${#job_ids[@]} == 0 )); then
  echo "ERROR: simulation job が1件も投入されていません"
  exit 1
fi

# =========================================
# 集計 job を依存関係付きで投入
# =========================================
dependency_job_ids=$(IFS=:; echo "${job_ids[*]}")

aggregate_job_name="aggregate_s${SCENARIO_FROM}_to_s${SCENARIO_TO}"
aggregate_job_script="${LOG_DIR}/aggregate_after_jobs_s${SCENARIO_FROM}_to_s${SCENARIO_TO}.sh"

cat > "${aggregate_job_script}" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "${SCRIPT_DIR}"

echo "[INFO] aggregate job started"
echo "[INFO] working directory: \$(pwd)"
echo "[INFO] expected branch: ${GIT_BRANCH_NAME}"

# -----------------------------------------
# branch 確認
# aggregate job 側では checkout しない
# 同じ作業ディレクトリで別 branch に切り替えられていたら止める
# -----------------------------------------
if [[ "${AUTO_GIT_PUSH}" == "1" ]]; then
  current_branch="\$(git rev-parse --abbrev-ref HEAD)"

  if [[ "\${current_branch}" != "${GIT_BRANCH_NAME}" ]]; then
    echo "ERROR: aggregate job 実行時の branch が想定と異なります"
    echo "expected: ${GIT_BRANCH_NAME}"
    echo "actual: \${current_branch}"
    echo "同じ作業ディレクトリで別ブランチに切り替えた可能性があります"
    echo "安全のため、集計・commit・push を中止します"
    exit 1
  fi
fi

# -----------------------------------------
# 古い同一 scenario の集計結果を削除
# -----------------------------------------
echo "[INFO] remove old aggregated outputs for target scenarios"

for (( scenarioID=${SCENARIO_FROM}; scenarioID<=${SCENARIO_TO}; scenarioID++ )); do
  rm -rf "${AGGREGATED_OUTPUT_DIR}/scenario\${scenarioID}"
done

# -----------------------------------------
# 集計を実行
# -----------------------------------------
echo "[INFO] run aggregation"
python3 "${AGGREGATE_SCRIPT_PATH}" "${LOG_DIR}" --output-dir "${AGGREGATED_OUTPUT_DIR}"

echo "[INFO] aggregation finished"
echo "[INFO] aggregated output dir: ${AGGREGATED_OUTPUT_DIR}"

# -----------------------------------------
# 集計後の生成物を push
# -----------------------------------------
if [[ "${AUTO_GIT_PUSH}" == "1" ]]; then
  echo "[INFO] git add aggregated results"

  # output.json / CDF 図などの集計結果を追加
  # .gitignore に引っかかる可能性があるため -f を付ける
  git add -f "${AGGREGATED_OUTPUT_DIR}"

  if git diff --cached --quiet; then
    echo "[INFO] commit 対象の集計結果はありません"
  else
    git commit -m "Add aggregated simulation results"
  fi

  echo "[INFO] push branch: ${GIT_REMOTE}/${GIT_BRANCH_NAME}"
  git push -u "${GIT_REMOTE}" "${GIT_BRANCH_NAME}"
else
  echo "[INFO] AUTO_GIT_PUSH=0 のため git add / commit / push はスキップします"
fi

echo "[INFO] aggregate job finished"
EOF

chmod +x "${aggregate_job_script}"

echo "[INFO] aggregation job を投入します"
echo "[INFO] dependency=${AGGREGATE_DEPENDENCY_TYPE}:${dependency_job_ids}"
echo "[INFO] aggregate job script=${aggregate_job_script}"

aggregate_job_id=$(
  sbatch \
    --parsable \
    --job-name="${aggregate_job_name}" \
    --dependency="${AGGREGATE_DEPENDENCY_TYPE}:${dependency_job_ids}" \
    --chdir="${SCRIPT_DIR}" \
    --output="${LOG_DIR}/%x_%j.out" \
    --error="${LOG_DIR}/%x_%j.err" \
    "${aggregate_job_script}"
)

aggregate_job_id="${aggregate_job_id%%;*}"

echo "[INFO] aggregation job submitted: ${aggregate_job_name} -> job_id=${aggregate_job_id}"
echo "All simulation jobs were submitted."
echo "Aggregation will start automatically after all simulation jobs finish."
echo "Aggregated results will be committed and pushed after aggregation."