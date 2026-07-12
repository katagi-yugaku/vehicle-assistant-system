#!/usr/bin/env bash
set -uo pipefail

usage() {
  cat <<'USAGE'
Usage:
  run_schubert_single.sh SCENARIO_ID EARLY_RATE V2V_RATE RUN_ID

Environment variables:
  PROJECT_ROOT        Override the detected Git/project root.
  ALLOW_NON_SCHUBERT  Set to 1 to allow execution on hosts other than schubert.
  FORCE_RERUN         Set to 1 to rerun an already successful job.
USAGE
}

is_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

is_number() {
  [[ "$1" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]]
}

find_project_root() {
  local candidate=""
  local current=""

  if [[ -n "${PROJECT_ROOT:-}" ]]; then
    candidate="$(cd -- "${PROJECT_ROOT}" 2>/dev/null && pwd -P)" || return 1
    if [[ -d "${candidate}/scenarios/JIP/1-1-4/simulation" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
    return 1
  fi

  if command -v git >/dev/null 2>&1; then
    candidate="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
    if [[ -n "${candidate}" && -d "${candidate}/scenarios/JIP/1-1-4/simulation" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  fi

  current="${SCRIPT_DIR}"
  while [[ "${current}" != "/" ]]; do
    if [[ -d "${current}/scenarios/JIP/1-1-4/simulation" ]]; then
      printf '%s\n' "${current}"
      return 0
    fi
    current="$(dirname -- "${current}")"
  done

  return 1
}

make_backup_path() {
  local source_dir="$1"
  local timestamp=""
  local base=""
  local candidate=""
  local suffix=1

  timestamp="$(date '+%Y%m%d_%H%M%S')"
  base="${source_dir}.backup_${timestamp}"
  candidate="${base}"

  while [[ -e "${candidate}" ]]; do
    candidate="${base}_${suffix}"
    ((suffix += 1))
  done

  printf '%s\n' "${candidate}"
}

write_status_file() {
  local exit_code="$1"
  local status="$2"
  local end_time="$3"
  local reason="${4:-}"
  local tmp_file="${STATUS_FILE}.tmp.$$"

  {
    printf 'job_name=%s\n' "${JOB_NAME}"
    printf 'hostname=%s\n' "${HOST_FQDN}"
    printf 'scenario_id=%s\n' "${SCENARIO_ID}"
    printf 'early_rate=%s\n' "${EARLY_RATE}"
    printf 'v2v_rate=%s\n' "${V2V_RATE}"
    printf 'run_id=%s\n' "${RUN_ID}"
    printf 'config=%s\n' "${CONFIG_PATH}"
    printf 'start_time=%s\n' "${START_TIME}"
    printf 'end_time=%s\n' "${end_time}"
    printf 'exit_code=%s\n' "${exit_code}"
    printf 'status=%s\n' "${status}"
    if [[ -n "${reason}" ]]; then
      printf 'reason=%s\n' "${reason}"
    fi
  } > "${tmp_file}"

  mv -f -- "${tmp_file}" "${STATUS_FILE}"
}

finish_job() {
  local exit_code="$1"
  local status="$2"
  local reason="${3:-}"
  local end_time=""
  local end_epoch=0
  local elapsed=0

  end_time="$(date '+%Y-%m-%dT%H:%M:%S%z')"
  end_epoch="$(date +%s)"
  elapsed=$((end_epoch - START_EPOCH))

  write_status_file "${exit_code}" "${status}" "${end_time}" "${reason}"

  printf '\n=== Job Result ===\n'
  printf 'end=%s\n' "${end_time}"
  printf 'elapsed_seconds=%s\n' "${elapsed}"
  printf 'exit_code=%s\n' "${exit_code}"
  printf 'status=%s\n' "${status}"
  if [[ -n "${reason}" ]]; then
    printf 'reason=%s\n' "${reason}"
  fi
}

terminate_simulation() {
  local signal_name="${1:-TERM}"

  if [[ -z "${SIM_PID:-}" ]]; then
    return 0
  fi

  if [[ "${SIM_USES_SETSID:-0}" == "1" ]]; then
    kill -s "${signal_name}" -- "-${SIM_PID}" 2>/dev/null || true
  else
    pkill -s "${signal_name}" -P "${SIM_PID}" 2>/dev/null || true
    kill -s "${signal_name}" "${SIM_PID}" 2>/dev/null || true
  fi
}

handle_signal() {
  local signal_name="$1"
  local exit_code=143

  trap - INT TERM

  if [[ "${signal_name}" == "INT" ]]; then
    exit_code=130
  fi

  printf 'Received %s; terminating simulation.\n' "${signal_name}" >&2
  terminate_simulation TERM

  if [[ -n "${SIM_PID:-}" ]]; then
    wait "${SIM_PID}" 2>/dev/null || true
    SIM_PID=""
  fi

  finish_job "${exit_code}" "FAILED" "interrupted_by_${signal_name}"
  exit "${exit_code}"
}

if (( $# != 4 )); then
  usage >&2
  exit 2
fi

SCENARIO_ID="$1"
EARLY_RATE="$2"
V2V_RATE="$3"
RUN_ID="$4"

if ! is_positive_integer "${SCENARIO_ID}"; then
  echo "ERROR: SCENARIO_ID must be a positive integer: ${SCENARIO_ID}" >&2
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
  echo "ERROR: RUN_ID must be a positive integer: ${RUN_ID}" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if ! PROJECT_ROOT="$(find_project_root)"; then
  echo "ERROR: Could not locate the vehicle-assistant-system project root." >&2
  echo "Set PROJECT_ROOT explicitly if the scripts are stored outside the repository." >&2
  exit 2
fi

HOST_FQDN="$(hostname 2>/dev/null || printf 'unknown')"
HOST_SHORT="$(hostname -s 2>/dev/null || printf '%s' "${HOST_FQDN%%.*}")"
if [[ "${HOST_SHORT%%.*}" != "schubert" && "${ALLOW_NON_SCHUBERT:-0}" != "1" ]]; then
  echo "ERROR: This script is intended for schubert; current host is ${HOST_FQDN}." >&2
  echo "Set ALLOW_NON_SCHUBERT=1 only for a deliberate test run." >&2
  exit 69
fi

MODULE_DIR="${PROJECT_ROOT}/scenarios/JIP/1-1-4/simulation"
CONFIG_PATH="${PROJECT_ROOT}/scenarios/JIP/configs/config_scenario_${SCENARIO_ID}.toml"
LOG_ROOT="${PROJECT_ROOT}/schubert_logs"
JOB_NAME="va_s${SCENARIO_ID}_e${EARLY_RATE}_v${V2V_RATE}_r${RUN_ID}"
RUN_DIR="${LOG_ROOT}/${JOB_NAME}"
STATUS_FILE="${RUN_DIR}/status.txt"
OUT_FILE="${RUN_DIR}/${JOB_NAME}.out"
ERR_FILE="${RUN_DIR}/${JOB_NAME}.err"
TRIPINFO_FILE="${RUN_DIR}/tripinfo.xml"

if ! mkdir -p -- "${LOG_ROOT}"; then
  echo "ERROR: Cannot create log root: ${LOG_ROOT}" >&2
  exit 73
fi
if [[ ! -w "${LOG_ROOT}" ]]; then
  echo "ERROR: Log root is not writable: ${LOG_ROOT}" >&2
  exit 73
fi

if [[ -f "${STATUS_FILE}" ]] && grep -Eq '^status=SUCCESS$' "${STATUS_FILE}" && [[ "${FORCE_RERUN:-0}" != "1" ]]; then
  echo "SKIP: ${JOB_NAME} already completed successfully."
  exit 0
fi

if [[ -e "${RUN_DIR}" ]]; then
  BACKUP_DIR="$(make_backup_path "${RUN_DIR}")"
  if ! mv -- "${RUN_DIR}" "${BACKUP_DIR}"; then
    echo "ERROR: Could not back up existing run directory: ${RUN_DIR}" >&2
    exit 73
  fi
  echo "Existing result moved to: ${BACKUP_DIR}"
fi

if ! mkdir -p -- "${RUN_DIR}"; then
  echo "ERROR: Cannot create run directory: ${RUN_DIR}" >&2
  exit 73
fi
if [[ ! -w "${RUN_DIR}" ]]; then
  echo "ERROR: Run directory is not writable: ${RUN_DIR}" >&2
  exit 73
fi

# Create all required per-job files immediately. SUMO overwrites tripinfo.xml.
: > "${OUT_FILE}"
: > "${ERR_FILE}"
: > "${TRIPINFO_FILE}"

exec > "${OUT_FILE}" 2> "${ERR_FILE}"

START_TIME="$(date '+%Y-%m-%dT%H:%M:%S%z')"
START_EPOCH="$(date +%s)"
SIM_PID=""
SIM_USES_SETSID=0

trap 'handle_signal INT' INT
trap 'handle_signal TERM' TERM

write_status_file "-1" "RUNNING" "" ""

printf '=== Job Info ===\n'
printf 'job_name=%s\n' "${JOB_NAME}"
printf 'host=%s\n' "${HOST_FQDN}"
printf 'pid=%s\n' "$$"
printf 'scenario=%s\n' "${SCENARIO_ID}"
printf 'early_rate=%s\n' "${EARLY_RATE}"
printf 'v2v_rate=%s\n' "${V2V_RATE}"
printf 'run_id=%s\n' "${RUN_ID}"
printf 'config=%s\n' "${CONFIG_PATH}"
printf 'output_dir=%s\n' "${RUN_DIR}"
printf 'start=%s\n' "${START_TIME}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 is not available in PATH." >&2
  finish_job 127 "FAILED" "python3_not_found"
  exit 127
fi
if ! command -v sumo >/dev/null 2>&1; then
  echo "ERROR: sumo is not available in PATH." >&2
  finish_job 127 "FAILED" "sumo_not_found"
  exit 127
fi

PYTHON_VERSION="$(python3 --version 2>&1)"
SUMO_VERSION="$(sumo --version 2>&1 | sed -n '1p')"
printf 'python_version=%s\n' "${PYTHON_VERSION}"
printf 'sumo_version=%s\n' "${SUMO_VERSION}"

if [[ ! -d "${PROJECT_ROOT}" ]]; then
  echo "ERROR: Project root does not exist: ${PROJECT_ROOT}" >&2
  finish_job 72 "FAILED" "project_root_missing"
  exit 72
fi
if [[ ! -d "${MODULE_DIR}" ]]; then
  echo "ERROR: Python module directory does not exist: ${MODULE_DIR}" >&2
  finish_job 72 "FAILED" "module_directory_missing"
  exit 72
fi
if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "ERROR: Config file does not exist: ${CONFIG_PATH}" >&2
  finish_job 66 "FAILED" "config_missing"
  exit 66
fi

export RUN_ID
export SCENARIO_ID
export EARLY_RATE
export V2V_RATE
export JOB_NAME
export RUN_OUTPUT_DIR="${RUN_DIR}"

PYTHONPATH_VALUE="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

printf '\n=== Command ===\n'
printf 'cd %q\n' "${RUN_DIR}"
printf 'PYTHONPATH=%q python3 -W ignore::UserWarning -m scenarios.JIP.1-1-4.simulation.runner_simulator --nogui %q %q %q\n' \
  "${PYTHONPATH_VALUE}" "${CONFIG_PATH}" "${EARLY_RATE}" "${V2V_RATE}"

if command -v setsid >/dev/null 2>&1; then
  SIM_USES_SETSID=1
fi

(
  cd -- "${RUN_DIR}" || exit 111

  if [[ "${SIM_USES_SETSID}" == "1" ]]; then
    exec setsid env \
      "PYTHONPATH=${PYTHONPATH_VALUE}" \
      "RUN_ID=${RUN_ID}" \
      "SCENARIO_ID=${SCENARIO_ID}" \
      "EARLY_RATE=${EARLY_RATE}" \
      "V2V_RATE=${V2V_RATE}" \
      "JOB_NAME=${JOB_NAME}" \
      "RUN_OUTPUT_DIR=${RUN_OUTPUT_DIR}" \
      python3 -W ignore::UserWarning \
        -m scenarios.JIP.1-1-4.simulation.runner_simulator \
        --nogui \
        "${CONFIG_PATH}" \
        "${EARLY_RATE}" \
        "${V2V_RATE}"
  else
    exec env \
      "PYTHONPATH=${PYTHONPATH_VALUE}" \
      "RUN_ID=${RUN_ID}" \
      "SCENARIO_ID=${SCENARIO_ID}" \
      "EARLY_RATE=${EARLY_RATE}" \
      "V2V_RATE=${V2V_RATE}" \
      "JOB_NAME=${JOB_NAME}" \
      "RUN_OUTPUT_DIR=${RUN_OUTPUT_DIR}" \
      python3 -W ignore::UserWarning \
        -m scenarios.JIP.1-1-4.simulation.runner_simulator \
        --nogui \
        "${CONFIG_PATH}" \
        "${EARLY_RATE}" \
        "${V2V_RATE}"
  fi
) &
SIM_PID=$!
printf 'simulation_pid=%s\n' "${SIM_PID}"

wait "${SIM_PID}"
EXIT_CODE=$?
SIM_PID=""

if (( EXIT_CODE == 0 )) && [[ ! -s "${TRIPINFO_FILE}" ]]; then
  echo "ERROR: Simulation returned 0, but tripinfo.xml is missing or empty." >&2
  EXIT_CODE=65
fi

if (( EXIT_CODE == 0 )); then
  finish_job 0 "SUCCESS"
else
  finish_job "${EXIT_CODE}" "FAILED" "simulation_failed"
fi

exit "${EXIT_CODE}"
