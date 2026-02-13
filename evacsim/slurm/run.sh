#!/bin/bash

# ==== パラメタ ===== 
# 引数2つでシナリオを回すIDを指定
SCENARIO_NAME="$1"
SCENARIO_START="$2"
SCENARIO_END="$3"
RUNS_PER_SCENARIO=50
# python3 -m scenarios.pervehicle.map_one.simulation.runner_simulator --nogui scenarios/pervehicle/configs/config_scenario_1.toml 0.5

CONFIG_PATH="scenarios/${SCENARIO_NAME}/configs/config_scenario_${SCENARIO_ID}.toml"
RESULT_PATH="senarios/${SCENARIO_NAME}/results/scenario_${SCENARIO_ID}"
MODULE="scenarios.pervehicle.map_one.simulation.runner_simulator"