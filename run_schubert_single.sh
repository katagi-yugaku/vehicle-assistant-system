#!/usr/bin/env bash
set -uo pipefail

usage() {
  cat <<'USAGE'
Usage:
  run_schubert_single.sh \
    SCENARIO_ID EARLY_RATE V2V_RATE RUN_ID \
    TARGET_SCENARIO LOG_ROOT COMMUNICATION_MODE

Example:
  ./run_schubert_single.sh \
    70 1.0 1.0 1 \
    JIP/1-1-4-v2vonly \
    /home/katagi/vehicle-assistant-system/schubert_logs_1-1-4-v2vonly \
    none

COMMUNICATION_MODE:
  none       runnerへ--communication-modeを渡さない。
  v2v_only   runnerへ--communication-mode v2v_onlyを渡す。
USAGE
}

is_integer() {
  [[ "$1" =~ ^[0-9]+$ ]]
}

is_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

is_number() {
  [[ "$1" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]]
}

normalize_target_scenario() {
  local value="$1"

  value="${value#./}"
  value="${value#scenarios/}"
  value="${value%/}"

  if [[ "${value}" != JIP/* ]]; then
    value="JIP/${value}"
  fi

  if [[ ! "${value}" =~ ^JIP/[A-Za-z0-9._-]+$ ]]; then
    return 1
  fi

  printf '%s\n' "${value}"
}

find_project_root() {
  local script_dir="$1"
  local candidate=""
  local current=""

  if [[ -n "${PROJECT_ROOT:-}" ]]; then
    candidate="$(cd -- "${PROJECT_ROOT}" 2>/dev/null && pwd -P)" || return 1
    if [[ -d "${candidate}/scenarios/JIP/configs" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
    return 1
  fi

  if command -v git >/dev/null 2>&1; then
    candidate="$(git -C "${script_dir}" rev-parse --show-toplevel 2>/dev/null || true)"
    if [[ -n "${candidate}" && -d "${candidate}/scenarios/JIP/configs" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  fi

  current="${script_dir}"
  while [[ "${current}" != "/" ]]; do
    if [[ -d "${current}/scenarios/JIP/configs" ]]; then
      printf '%s\n' "${current}"
      return 0
    fi
    current="$(dirname -- "${current}")"
  done

  return 1
}

write_status() {
  local state="$1"
  local exit_code="$2"
  local end_time="$3"
  local temp_status="${STATUS_FILE}.tmp.$$"

  {
    printf 'status=%s\n' "${state}"
    printf 'exit_code=%s\n' "${exit_code}"
    printf 'job_name=%s\n' "${JOB_NAME}"
    printf 'scenario_id=%s\n' "${SCENARIO_ID}"
    printf 'early_rate=%s\n' "${EARLY_RATE}"
    printf 'v2v_rate=%s\n' "${V2V_RATE}"
    printf 'run_id=%s\n' "${RUN_ID}"
    printf 'target_scenario=%s\n' "${TARGET_SCENARIO}"
    printf 'runner_module=%s\n' "${RUNNER_MODULE}"
    printf 'communication_mode=%s\n' "${COMMUNICATION_MODE}"
    printf 'host=%s\n' "${HOST_NAME}"
    printf 'pid=%s\n' "$$"
    printf 'start_time=%s\n' "${START_TIME}"
    printf 'end_time=%s\n' "${end_time}"
  } > "${temp_status}"

  mv -f -- "${temp_status}" "${STATUS_FILE}"
}

cleanup() {
  local shell_exit=$?
  local end_time=""

  trap - EXIT INT TERM HUP

  if (( FINALIZED == 0 )); then
    end_time="$(date '+%Y-%m-%dT%H:%M:%S%z')"
    write_status "FAILED" "${shell_exit}" "${end_time}" 2>/dev/null || true
  fi

  rm -rf -- "${LOCK_DIR}" 2>/dev/null || true
  exit "${shell_exit}"
}

handle_signal() {
  local signal_name="$1"
  local signal_exit=143

  if [[ "${signal_name}" == "INT" ]]; then
    signal_exit=130
  fi

  if [[ -n "${SIMULATION_PID:-}" ]]; then
    kill -TERM "${SIMULATION_PID}" 2>/dev/null || true
    pkill -TERM -P "${SIMULATION_PID}" 2>/dev/null || true
  fi

  exit "${signal_exit}"
}

if (( $# != 7 )); then
  usage >&2
  exit 2
fi

SCENARIO_ID="$1"
EARLY_RATE="$2"
V2V_RATE="$3"
RUN_ID="$4"
TARGET_SCENARIO_RAW="$5"
LOG_ROOT="$6"
COMMUNICATION_MODE="$7"

if ! is_integer "${SCENARIO_ID}"; then
  echo "ERROR: SCENARIO_ID must be an integer: ${SCENARIO_ID}" >&2
  exit 2
fi
if ! is_number "${EARLY_RATE}"; then
  echo "ERROR: EARLY_RATE must be numeric: ${EARLY_RATE}" >&2
  exit 2
fi
if ! is_number "${V2V_RATE}"; then
  echo "ERROR: V2V_RATE must be numeric: ${V2V_RATE}" >&2
  exit 2
fi
if ! is_positive_integer "${RUN_ID}"; then
  echo "ERROR: RUN_ID must be an integer >= 1: ${RUN_ID}" >&2
  exit 2
fi
if ! TARGET_SCENARIO="$(normalize_target_scenario "${TARGET_SCENARIO_RAW}")"; then
  echo "ERROR: Invalid target scenario: ${TARGET_SCENARIO_RAW}" >&2
  exit 2
fi
if [[ "${COMMUNICATION_MODE}" != "none" ]] && \
   [[ ! "${COMMUNICATION_MODE}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "ERROR: Invalid communication mode: ${COMMUNICATION_MODE}" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if ! PROJECT_ROOT="$(find_project_root "${SCRIPT_DIR}")"; then
  echo "ERROR: Could not locate the vehicle-assistant-system project root." >&2
  exit 2
fi

if [[ "${LOG_ROOT}" != /* ]]; then
  LOG_ROOT="${PROJECT_ROOT}/${LOG_ROOT}"
fi

CONFIG="${PROJECT_ROOT}/scenarios/JIP/configs/config_scenario_${SCENARIO_ID}.toml"
RUNNER_PATH="${PROJECT_ROOT}/scenarios/${TARGET_SCENARIO}/simulation/runner_simulator.py"
RUNNER_MODULE="scenarios.${TARGET_SCENARIO//\//.}.simulation.runner_simulator"
JOB_NAME="va_s${SCENARIO_ID}_e${EARLY_RATE}_v${V2V_RATE}_r${RUN_ID}"
RUN_DIR="${LOG_ROOT}/${JOB_NAME}"
LOCK_DIR="${LOG_ROOT}/.${JOB_NAME}.lock"
OUT_FILE="${RUN_DIR}/${JOB_NAME}.out"
ERR_FILE="${RUN_DIR}/${JOB_NAME}.err"
STATUS_FILE="${RUN_DIR}/status.txt"
COMMAND_FILE="${RUN_DIR}/command.txt"
HOST_NAME="$(hostname 2>/dev/null || printf 'unknown')"
START_TIME="$(date '+%Y-%m-%dT%H:%M:%S%z')"
FINALIZED=0
SIMULATION_PID=""

if [[ ! -f "${CONFIG}" ]]; then
  echo "ERROR: Config file does not exist: ${CONFIG}" >&2
  exit 72
fi
if [[ ! -f "${RUNNER_PATH}" ]]; then
  echo "ERROR: runner_simulator.py does not exist: ${RUNNER_PATH}" >&2
  exit 72
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 is not available in PATH." >&2
  exit 127
fi
if ! command -v sumo >/dev/null 2>&1; then
  echo "ERROR: sumo is not available in PATH." >&2
  exit 127
fi

mkdir -p -- "${LOG_ROOT}"

if ! mkdir -- "${LOCK_DIR}" 2>/dev/null; then
  if [[ "${FORCE_UNLOCK:-0}" == "1" ]]; then
    echo "WARNING: Removing an existing lock: ${LOCK_DIR}" >&2
    rm -rf -- "${LOCK_DIR}"
    if ! mkdir -- "${LOCK_DIR}" 2>/dev/null; then
      echo "ERROR: Could not acquire lock after FORCE_UNLOCK: ${LOCK_DIR}" >&2
      exit 75
    fi
  else
    echo "ERROR: Another process may be running this job: ${LOCK_DIR}" >&2
    echo "Set FORCE_UNLOCK=1 only after confirming that no process is active." >&2
    exit 75
  fi
fi

trap cleanup EXIT
trap 'handle_signal INT' INT
trap 'handle_signal TERM' TERM
trap 'handle_signal HUP' HUP

# 既存結果を上書きせず、再実行時にはバックアップする。
if [[ -d "${RUN_DIR}" ]]; then
  if [[ "${FORCE_RERUN:-0}" != "1" ]] && \
     [[ -f "${STATUS_FILE}" ]] && \
     grep -Eq '^status=SUCCESS$' "${STATUS_FILE}"; then
    echo "SKIP: Successful result already exists: ${RUN_DIR}"
    FINALIZED=1
    rm -rf -- "${LOCK_DIR}"
    trap - EXIT INT TERM HUP
    exit 0
  fi

  backup_suffix="$(date '+%Y%m%d_%H%M%S')_$$"
  backup_dir="${RUN_DIR}.backup_${backup_suffix}"
  mv -- "${RUN_DIR}" "${backup_dir}"
  echo "INFO: Existing result moved to: ${backup_dir}"
fi

mkdir -p -- "${RUN_DIR}"
write_status "RUNNING" "-1" ""

command=(
  python3
  -W ignore::UserWarning
  -m "${RUNNER_MODULE}"
  --nogui
)

if [[ "${COMMUNICATION_MODE}" != "none" ]]; then
  command+=(--communication-mode "${COMMUNICATION_MODE}")
fi

command+=(
  "${CONFIG}"
  "${EARLY_RATE}"
  "${V2V_RATE}"
)

{
  printf 'cd %q\n' "${RUN_DIR}"
  printf 'PYTHONPATH=%q ' "${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
  printf '%q ' "${command[@]}"
  printf '\n'
} > "${COMMAND_FILE}"

# runnerが相対パスで出力するtripinfo.xml等をジョブごとに分離する。
cd "${RUN_DIR}"
export RUN_ID
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

{
  echo "=== Job Info ==="
  echo "host              : ${HOST_NAME}"
  echo "pid               : $$"
  echo "job_name          : ${JOB_NAME}"
  echo "scenario          : ${SCENARIO_ID}"
  echo "early_rate        : ${EARLY_RATE}"
  echo "v2v_rate          : ${V2V_RATE}"
  echo "run_id            : ${RUN_ID}"
  echo "target_scenario   : ${TARGET_SCENARIO}"
  echo "runner_module     : ${RUNNER_MODULE}"
  echo "communication_mode: ${COMMUNICATION_MODE}"
  echo "config            : ${CONFIG}"
  echo "run_dir           : ${RUN_DIR}"
  echo "start             : ${START_TIME}"
  echo

  "${command[@]}" &
  SIMULATION_PID=$!
  wait "${SIMULATION_PID}"
} > "${OUT_FILE}" 2> "${ERR_FILE}"
SIMULATION_EXIT=$?
SIMULATION_PID=""

END_TIME="$(date '+%Y-%m-%dT%H:%M:%S%z')"

if (( SIMULATION_EXIT == 0 )); then
  {
    echo
    echo "end               : ${END_TIME}"
    echo "exit_code         : ${SIMULATION_EXIT}"
  } >> "${OUT_FILE}"

  write_status "SUCCESS" "${SIMULATION_EXIT}" "${END_TIME}"
  FINALIZED=1
  rm -rf -- "${LOCK_DIR}"
  trap - EXIT INT TERM HUP
  exit 0
fi

{
  echo
  echo "end               : ${END_TIME}"
  echo "exit_code         : ${SIMULATION_EXIT}"
} >> "${ERR_FILE}"

write_status "FAILED" "${SIMULATION_EXIT}" "${END_TIME}"
FINALIZED=1
rm -rf -- "${LOCK_DIR}"
trap - EXIT INT TERM HUP
exit "${SIMULATION_EXIT}"