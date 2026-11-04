#!/usr/bin/env bash
# 実験の投入。使い方は tools/run_experiment.py --help
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec "${EVAC_PYTHON:-python3}" "${HERE}/run_experiment.py" "$@"
