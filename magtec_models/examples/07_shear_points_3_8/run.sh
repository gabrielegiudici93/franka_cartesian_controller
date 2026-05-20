#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 src/franka_controller/franka_shear_test_points_3_8_fixed_1mm.py "$@"
