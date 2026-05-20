#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 src/franka_controller/franka_skin_test_multiple_points.py "$@"
