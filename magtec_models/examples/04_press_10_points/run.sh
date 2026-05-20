#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 src/franka_controller/franka_10_random_points.py "$@"
