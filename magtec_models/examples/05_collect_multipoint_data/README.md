# Example 05 — Multi-Point Data Collection (Normal Forces)

Full collection: robot + FT + magnetic skin → HDF5 under `data/Multiple_Points/`.

This runs `franka_skin_test_multiple_points.py`, which configures the point grid and delegates to the core engine **`franka_skin_test.py`** (see [magtec_models/README.md](../../README.md#data-collection-franka_skin_testpy-core-engine)).

## Before running

1. `config/hardware.yaml` configured
2. Skin calibrated (example 01)
3. Robot unlocked, FCI active — [../../../docs/ROBOT_CONNECTION.md](../../../docs/ROBOT_CONNECTION.md)

## Run

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/franka_controller/franka_skin_test_multiple_points.py
```

Follow terminal prompts for stretch levels (0% / 10% / 20% by default).

## Output

`data/Multiple_Points/<run_label>_000pct.h5`, etc.

Train with example 08.
