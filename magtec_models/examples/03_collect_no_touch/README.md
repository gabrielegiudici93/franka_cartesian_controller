# Example 03 — No-Touch Baseline Collection

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/franka_controller/collect_no_touch_data.py \
  --stretch 0 10 20 \
  --data-dir data/Multiple_Points \
  --run-label no_touch_$(date +%Y%m%d_%H%M%S)
```
