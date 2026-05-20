# Example 06 — Shear Test Quick (10 Points, No Save)

Validates shear motion on 10 points: 4 directions, 1 repetition each, no HDF5.
Does **not** connect to magnetic sensor (compatible with live GUI).

## Run

```bash
cd ~/franka_magtec_workspace/magtec_models
./examples/06_shear_test_quick/run.sh
```

## Full shear data collection (with save)

```bash
python3 src/franka_controller/franka_skin_test_shear_forces.py
```
