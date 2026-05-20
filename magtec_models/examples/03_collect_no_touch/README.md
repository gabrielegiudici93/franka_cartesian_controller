# Example 03 — No-Touch Baseline Collection

Records magnetic sensor data **without** moving the robot. Use before/after skin changes.

## Run

From the repo root, after activating the env (`conda activate franka_interface`):

```bash
cd magtec_models
./examples/03_collect_no_touch/run.sh
```

Output: `data/Multiple_Points/no_touch_<timestamp>/`

## Customize

Edit `run.sh` for stretch levels and `--run-label`.
