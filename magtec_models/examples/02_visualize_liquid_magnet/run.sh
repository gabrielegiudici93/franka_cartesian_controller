#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 src/validation_tests/liquid_magnet_15_taxels_sensorreader.py "$@"
