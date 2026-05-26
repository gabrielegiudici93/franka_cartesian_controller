# Example 05 — Multi-Point Data Collection (Normal Forces)

Full collection: robot + FT + magnetic skin → HDF5 under `data/Multiple_Points/`.

This runs `franka_skin_test_multiple_points.py`, which configures the point grid and delegates to the core engine **`franka_skin_test.py`**.

**Full step-by-step guide (recommended):** [docs/guides/DATA_COLLECTION.md](../../docs/guides/DATA_COLLECTION.md)

## Before running

1. `config/hardware.yaml` configured
2. Skin calibrated (example 01) — **close visualization before starting**
3. Robot unlocked, FCI active — [../../../docs/ROBOT_CONNECTION.md](../../../docs/ROBOT_CONNECTION.md)
4. Edit collection settings in `src/franka_controller/franka_skin_test_multiple_points.py` if needed (`STRETCH_LEVELS`, `TARGET_POSITION_COORDS`, etc.)

## Run

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/franka_controller/franka_skin_test_multiple_points.py
```

You will be asked for a **run label** (folder name under `data/Multiple_Points/`), then prompted before each stretch level.

## Verify output (required)

```bash
ls -lh data/Multiple_Points/YOUR_RUN_LABEL/
```

The folder must contain `.h5` files. If it is empty, see the troubleshooting section in [DATA_COLLECTION.md](../../docs/guides/DATA_COLLECTION.md).

## Train

Example 08 — `train_best_models.py` with `--normal-dir` pointing at your run folder.
