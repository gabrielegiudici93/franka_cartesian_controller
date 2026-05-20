#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 src/validation_tests/15_taxels_visualization.py "$@"
