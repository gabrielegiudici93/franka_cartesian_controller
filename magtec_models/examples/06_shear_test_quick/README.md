# Example 06 — Shear Test Quick (10 Points, No Save)

Validates shear motion on 10 points: 4 directions, 1 repetition each, no HDF5.
Does **not** connect to magnetic sensor (compatible with live GUI).

## Run

From the repo root, with the env active and the robot reachable (see [`docs/ROBOT_CONNECTION.md`](../../../docs/ROBOT_CONNECTION.md)):

```bash
cd magtec_models
./examples/06_shear_test_quick/run.sh
```

## Full shear data collection (with save)

```bash
python3 src/franka_controller/franka_skin_test_shear_forces.py
```
