# Example 08 — Train Models

Edit paths, then:

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/training/train_best_models.py \
  --normal-dir data/Multiple_Points/YOUR_NORMAL_RUN \
  --run-label collab_demo_models \
  --remove-outliers

# optional shear data:
python3 src/training/train_best_models.py \
  --normal-dir data/Multiple_Points/YOUR_NORMAL_RUN \
  --shear-dir data/Multiple_Points/YOUR_SHEAR_RUN \
  --run-label collab_demo_models \
  --remove-outliers
```
