#!/bin/bash

set -eu

LOG_DIR="/Users/kashiisamutakeshi/vehicle-assistant-system/logs"

for f in "$LOG_DIR"/*.out; do
  echo "processing: $f"
  python3 results_save_data.py "$f"
done

