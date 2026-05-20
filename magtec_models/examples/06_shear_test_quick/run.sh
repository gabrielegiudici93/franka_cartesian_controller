#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 src/franka_controller/franka_shear_test_10_points_quick.py "$@"
