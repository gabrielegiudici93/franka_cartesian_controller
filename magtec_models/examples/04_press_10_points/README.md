# Example 04 — Press 10 Points (FT Only)

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/franka_controller/franka_10_random_points.py
```

Press Enter between points. Ctrl+C to stop.
