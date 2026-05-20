#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
RUN_LABEL="no_touch_$(date +%Y%m%d_%H%M%S)"
python3 src/franka_controller/collect_no_touch_data.py \
  --stretch 0 10 20 \
  --data-dir data/Multiple_Points \
  --run-label "$RUN_LABEL"
