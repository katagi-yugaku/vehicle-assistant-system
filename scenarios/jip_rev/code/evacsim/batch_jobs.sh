#!/usr/bin/env bash
set -euo pipefail

# ===== プロジェクトルートを取得 =====
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# ===== 実行パラメータ =====
CONFIG="$ROOT_DIR/scenarios/pervehicle/configs/config_scenario_1.toml"
SCRIPT="$ROOT_DIR/scenarios/pervehicle/map_one/simulation/runner_simulator.py"

EARLY_RATE="${1:-0.5}"
LATE_RATE="${2:-1.0}"

# ===== 実行 =====
PYTHONPATH="$ROOT_DIR" python3 "$SCRIPT" \
  --nogui "$CONFIG" "$EARLY_RATE" "$LATE_RATE"

