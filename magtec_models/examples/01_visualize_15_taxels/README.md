# Example 01 — 15 Taxels Visualization

Live 3×5 taxel GUI (Fx/Fy movement, Fz radius).

## Prerequisites

- Magnetic skin connected (`STRETCHMAGTEC_PORT` in `config/hardware.yaml`)
- No other script using the same serial port

## Run

From the repo root, after activating the env (`conda activate franka_interface`):

```bash
cd magtec_models
./examples/01_visualize_15_taxels/run.sh
```

Or call the script directly:

```bash
cd magtec_models
python3 src/validation_tests/15_taxels_visualization.py
```

## Extended version (XYZ plots + MP4 recording)

```bash
python3 src/validation_tests/15_taxels_visualization_plus.py
```

Press `V` to toggle window recording.
