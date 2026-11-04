#!/usr/bin/env bash
# usage: runrev.sh CODE_ROOT WORKDIR SEED CONFIG_PATH LAYOUT [extra args]
set -u
CODE=$1; WD=$2; SEED=$3; CFG=$4; LAY=$5; shift 5
export SUMO_HOME=/home/claude/venv310/lib/python3.10/site-packages/sumo
export PYTHONHASHSEED=0 PYTHONPATH=$CODE
mkdir -p $WD && cd $WD
start=$(date +%s)
/home/claude/venv310/bin/python -W ignore::UserWarning $CODE/tools/compat/run_seeded.py $SEED $WD/trace.json scenarios.JIP.$LAY.simulation.runner_simulator --nogui $CFG 1.0 1.0 --seed $SEED --raw-output $WD/raw.json "$@" > $WD/run.out 2> $WD/run.err
rc=$?
echo "exit=$rc elapsed=$(( $(date +%s)-start ))s" > $WD/status.txt
