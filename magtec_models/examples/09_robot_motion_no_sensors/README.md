# Example 09 — Robot Motion (No Sensors)

Franka grid motions **without** magnetic skin or FT sensor.

## Run

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/franka_controller/franka_motion_test_no_sensors.py
python3 src/franka_controller/franka_motion_test_no_sensors.py --points 1 2 3 --indent-mm 0.5 --no-prompt
python3 src/franka_controller/franka_motion_test_no_sensors.py --all
```

Options: `--no-press`, `--no-prompt`, `--indent-mm`, `--points`, `--all`.
