#!/usr/bin/env bash
# 実行中の実験の進み具合: tools/experiment_status.sh runs/<実験ID>
RUN_ROOT="${1:?RUN_ROOT is required}"
PLANNED=$(( $(wc -l < "${RUN_ROOT}/tasks.tsv") - 1 ))
OK=$(find "${RUN_ROOT}" -name status.txt -path '*/r[0-9]*' -exec grep -l '^status=SUCCESS$' {} + 2>/dev/null | wc -l)
NG=$(find "${RUN_ROOT}" -name status.txt -path '*/r[0-9]*' -exec grep -L '^status=SUCCESS$' {} + 2>/dev/null | wc -l)
echo "$(basename "${RUN_ROOT}"): success ${OK} / failed ${NG} / planned ${PLANNED} (running or waiting $(( PLANNED - OK - NG )))"
command -v squeue >/dev/null && squeue -u "$USER" -h -o '%.18i %.20j %.8T %.10M %R' | grep -E 'va_' || true
