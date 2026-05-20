#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."

# --- EDIT THESE PATHS ---
NORMAL_DIR="data/Multiple_Points/YOUR_NORMAL_RUN"
SHEAR_DIR="data/Multiple_Points/YOUR_SHEAR_RUN"
RUN_LABEL="collab_demo_models"
# ------------------------

if [[ ! -d "$NORMAL_DIR" ]]; then
  echo "Set NORMAL_DIR in run.sh to an existing data folder."
  echo "Example: data/Multiple_Points/2.5mm_single_test42"
  exit 1
fi

ARGS=(--normal-dir "$NORMAL_DIR" --run-label "$RUN_LABEL" --remove-outliers)
if [[ -d "$SHEAR_DIR" ]]; then
  ARGS+=(--shear-dir "$SHEAR_DIR")
fi

python3 src/training/train_best_models.py "${ARGS[@]}" "$@"
