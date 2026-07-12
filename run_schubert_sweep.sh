#!/usr/bin/env bash
set -uo pipefail

usage() {
  cat <<'USAGE'
Usage:
  run_schubert_sweep.sh SCENARIO_FROM SCENARIO_TO [MAX_PARALLEL]

Examples:
  ./run_schubert_sweep.sh 1 36
  MAX_PARALLEL=16 ./run_schubert_sweep.sh 1 36
  ./run_schubert_sweep.sh 1 36 16

Environment variables:
  N                    Runs per parameter combination; default: 50
  MAX_PARALLEL         Maximum concurrent jobs; default: 20
  EARLY_RATE_LIST      Space-separated early-rate values; default: "1.0"
  V2V_RATE_LIST        Space-separated V2V-rate values; default: "1.0"
  PROJECT_ROOT         Override the detected Git/project root.
  ALLOW_NON_SCHUBERT   Set to 1 to allow execution on hosts other than schubert.
  FORCE_RERUN          Set to 1 to rerun successful jobs.
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

status_is_success() {
  local status_file="$1"
  [[ -f "${status_file}" ]] && grep -Eq '^status=SUCCESS$' "${status_file}"
}

record_result() {
  local job_name="$1"
  local exit_code="$2"

  if (( exit_code == 0 )); then
    ((SUCCESS_THIS_RUN += 1))
  else
    ((FAILED_COUNT += 1))
    FAILED_JOBS+=("${job_name} (exit=${exit_code})")
  fi
}

reap_one() {
  local finished_pid=""
  local exit_code=0
  local job_name="unknown"

  wait -n -p finished_pid
  exit_code=$?

  if [[ -z "${finished_pid}" ]]; then
    echo "ERROR: wait -n returned without a PID while jobs were expected." >&2
    return 1
  fi

  job_name="${PID_TO_JOB[${finished_pid}]:-unknown_pid_${finished_pid}}"
  unset 'PID_TO_JOB['"${finished_pid}"']'
  ((RUNNING_COUNT -= 1))

  record_result "${job_name}" "${exit_code}"

  if (( exit_code == 0 )); then
    printf '[DONE]   %s\n' "${job_name}"
  else
    printf '[FAILED] %s exit=%s\n' "${job_name}" "${exit_code}" >&2
  fi
}

print_summary() {
  local end_time="$1"
  local successful_total=$((SUCCESS_THIS_RUN + SKIPPED_COUNT))
  local not_launched=$((TOTAL_EXPERIMENTS - LAUNCHED_COUNT - SKIPPED_COUNT))

  printf '\n=== Sweep Summary ===\n'
  printf '総実験数=%s\n' "${TOTAL_EXPERIMENTS}"
  printf '投入数=%s\n' "${LAUNCHED_COUNT}"
  printf '未投入数=%s\n' "${not_launched}"
  printf '成功数=%s\n' "${successful_total}"
  printf '  今回成功=%s\n' "${SUCCESS_THIS_RUN}"
  printf '  成功済みスキップ=%s\n' "${SKIPPED_COUNT}"
  printf '失敗数=%s\n' "${FAILED_COUNT}"
  printf '開始時刻=%s\n' "${SWEEP_START_TIME}"
  printf '終了時刻=%s\n' "${end_time}"

  if (( FAILED_COUNT > 0 )); then
    printf '\n=== Failed Jobs ===\n'
    printf '%s\n' "${FAILED_JOBS[@]}"
  fi
}

handle_sweep_signal() {
  local signal_name="$1"
  local exit_code=143
  local pid=""
  local job_name=""
  local child_exit=0
  local deadline=0

  trap - INT TERM

  if [[ "${signal_name}" == "INT" ]]; then
    exit_code=130
  fi

  printf '\nReceived %s; terminating %s active job(s).\n' "${signal_name}" "${RUNNING_COUNT}" >&2

  for pid in "${!PID_TO_JOB[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done

  deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    mapfile -t active_pids < <(jobs -pr)
    (( ${#active_pids[@]} == 0 )) && break
    sleep 1
  done

  mapfile -t active_pids < <(jobs -pr)
  for pid in "${active_pids[@]}"; do
    pkill -KILL -P "${pid}" 2>/dev/null || true
    kill -KILL "${pid}" 2>/dev/null || true
  done

  for pid in "${!PID_TO_JOB[@]}"; do
    job_name="${PID_TO_JOB[${pid}]}"
    wait "${pid}" 2>/dev/null
    child_exit=$?
    unset 'PID_TO_JOB['"${pid}"']'
    ((RUNNING_COUNT -= 1))
    record_result "${job_name}" "${child_exit}"
  done

  wait 2>/dev/null || true
  print_summary "$(date '+%Y-%m-%dT%H:%M:%S%z')"
  exit "${exit_code}"
}

if (( $# < 2 || $# > 3 )); then
  usage >&2
  exit 2
fi

SCENARIO_FROM="$1"
SCENARIO_TO="$2"
MAX_PARALLEL="${3:-${MAX_PARALLEL:-20}}"
n="${N:-50}"

if ! is_integer "${SCENARIO_FROM}"; then
  echo "ERROR: SCENARIO_FROM must be an integer: ${SCENARIO_FROM}" >&2
  exit 2
fi
if ! is_integer "${SCENARIO_TO}"; then
  echo "ERROR: SCENARIO_TO must be an integer: ${SCENARIO_TO}" >&2
  exit 2
fi
if (( SCENARIO_FROM > SCENARIO_TO )); then
  echo "ERROR: SCENARIO_FROM must be less than or equal to SCENARIO_TO." >&2
  exit 2
fi
if ! is_positive_integer "${n}"; then
  echo "ERROR: N must be an integer greater than or equal to 1: ${n}" >&2
  exit 2
fi
if ! is_positive_integer "${MAX_PARALLEL}"; then
  echo "ERROR: MAX_PARALLEL must be an integer greater than or equal to 1: ${MAX_PARALLEL}" >&2
  exit 2
fi

# Defaults requested for the experiment. Override with space-separated environment variables.
early_rate_list=(1.0)
v2v_capable_vehicle_rate_list=(1.0)
if [[ -n "${EARLY_RATE_LIST:-}" ]]; then
  read -r -a early_rate_list <<< "${EARLY_RATE_LIST}"
fi
if [[ -n "${V2V_RATE_LIST:-}" ]]; then
  read -r -a v2v_capable_vehicle_rate_list <<< "${V2V_RATE_LIST}"
fi

if (( ${#early_rate_list[@]} == 0 )); then
  echo "ERROR: EARLY_RATE_LIST produced an empty list." >&2
  exit 2
fi
if (( ${#v2v_capable_vehicle_rate_list[@]} == 0 )); then
  echo "ERROR: V2V_RATE_LIST produced an empty list." >&2
  exit 2
fi
for value in "${early_rate_list[@]}"; do
  if ! is_number "${value}"; then
    echo "ERROR: Invalid early_rate value: ${value}" >&2
    exit 2
  fi
done
for value in "${v2v_capable_vehicle_rate_list[@]}"; do
  if ! is_number "${value}"; then
    echo "ERROR: Invalid v2v_rate value: ${value}" >&2
    exit 2
  fi
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SINGLE_SCRIPT="${SCRIPT_DIR}/run_schubert_single.sh"

if [[ ! -f "${SINGLE_SCRIPT}" ]]; then
  echo "ERROR: run_schubert_single.sh does not exist: ${SINGLE_SCRIPT}" >&2
  exit 2
fi
if [[ ! -r "${SINGLE_SCRIPT}" ]]; then
  echo "ERROR: run_schubert_single.sh is not readable by bash: ${SINGLE_SCRIPT}" >&2
  exit 2
fi

if ! PROJECT_ROOT="$(find_project_root)"; then
  echo "ERROR: Could not locate the vehicle-assistant-system project root." >&2
  echo "Set PROJECT_ROOT explicitly if the scripts are stored outside the repository." >&2
  exit 2
fi

MODULE_DIR="${PROJECT_ROOT}/scenarios/JIP/1-1-4/simulation"
LOG_ROOT="${PROJECT_ROOT}/schubert_logs"

HOST_FQDN="$(hostname 2>/dev/null || printf 'unknown')"
HOST_SHORT="$(hostname -s 2>/dev/null || printf '%s' "${HOST_FQDN%%.*}")"
if [[ "${HOST_SHORT%%.*}" != "schubert" && "${ALLOW_NON_SCHUBERT:-0}" != "1" ]]; then
  echo "ERROR: This script is intended for schubert; current host is ${HOST_FQDN}." >&2
  echo "Set ALLOW_NON_SCHUBERT=1 only for a deliberate test run." >&2
  exit 69
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 is not available in PATH." >&2
  exit 127
fi
if ! command -v sumo >/dev/null 2>&1; then
  echo "ERROR: sumo is not available in PATH." >&2
  exit 127
fi
if [[ ! -d "${PROJECT_ROOT}" ]]; then
  echo "ERROR: Project root does not exist: ${PROJECT_ROOT}" >&2
  exit 72
fi
if [[ ! -d "${MODULE_DIR}" ]]; then
  echo "ERROR: Python module directory does not exist: ${MODULE_DIR}" >&2
  exit 72
fi
if ! mkdir -p -- "${LOG_ROOT}"; then
  echo "ERROR: Cannot create schubert_logs: ${LOG_ROOT}" >&2
  exit 73
fi
if [[ ! -w "${LOG_ROOT}" ]]; then
  echo "ERROR: schubert_logs is not writable: ${LOG_ROOT}" >&2
  exit 73
fi

printf '=== Host Info ===\n'
hostname
uname -a
nproc
free -h
python3 --version
sumo --version

printf '\n=== Sweep Settings ===\n'
printf 'project_root=%s\n' "${PROJECT_ROOT}"
printf 'scenario_from=%s\n' "${SCENARIO_FROM}"
printf 'scenario_to=%s\n' "${SCENARIO_TO}"
printf 'n=%s\n' "${n}"
printf 'early_rate_list=%s\n' "${early_rate_list[*]}"
printf 'v2v_rate_list=%s\n' "${v2v_capable_vehicle_rate_list[*]}"
printf 'max_parallel=%s\n' "${MAX_PARALLEL}"
printf 'force_rerun=%s\n' "${FORCE_RERUN:-0}"

SWEEP_START_TIME="$(date '+%Y-%m-%dT%H:%M:%S%z')"
TOTAL_EXPERIMENTS=$((
  (SCENARIO_TO - SCENARIO_FROM + 1)
  * ${#early_rate_list[@]}
  * ${#v2v_capable_vehicle_rate_list[@]}
  * n
))
LAUNCHED_COUNT=0
RUNNING_COUNT=0
SUCCESS_THIS_RUN=0
SKIPPED_COUNT=0
FAILED_COUNT=0
declare -A PID_TO_JOB=()
declare -a FAILED_JOBS=()

trap 'handle_sweep_signal INT' INT
trap 'handle_sweep_signal TERM' TERM

for ((scenario_id = SCENARIO_FROM; scenario_id <= SCENARIO_TO; scenario_id++)); do
  for early_rate in "${early_rate_list[@]}"; do
    for v2v_rate in "${v2v_capable_vehicle_rate_list[@]}"; do
      for ((run_id = 1; run_id <= n; run_id++)); do
        job_name="va_s${scenario_id}_e${early_rate}_v${v2v_rate}_r${run_id}"
        status_file="${LOG_ROOT}/${job_name}/status.txt"

        if [[ "${FORCE_RERUN:-0}" != "1" ]] && status_is_success "${status_file}"; then
          ((SKIPPED_COUNT += 1))
          printf '[SKIP]   %s\n' "${job_name}"
          continue
        fi

        while (( RUNNING_COUNT >= MAX_PARALLEL )); do
          reap_one
        done

        bash "${SINGLE_SCRIPT}" \
          "${scenario_id}" \
          "${early_rate}" \
          "${v2v_rate}" \
          "${run_id}" &
        child_pid=$!

        PID_TO_JOB["${child_pid}"]="${job_name}"
        ((RUNNING_COUNT += 1))
        ((LAUNCHED_COUNT += 1))
        printf '[START]  pid=%s %s (%s/%s active)\n' \
          "${child_pid}" "${job_name}" "${RUNNING_COUNT}" "${MAX_PARALLEL}"
      done
    done
  done
done

while (( RUNNING_COUNT > 0 )); do
  reap_one
done

# All children have already been reaped above. Keep a final wait as an explicit safety net.
wait 2>/dev/null || true

SWEEP_END_TIME="$(date '+%Y-%m-%dT%H:%M:%S%z')"
print_summary "${SWEEP_END_TIME}"

if (( FAILED_COUNT > 0 )); then
  exit 1
fi
exit 0
