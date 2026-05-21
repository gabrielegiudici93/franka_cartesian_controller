# Example 07 — Shear on Taxels 3–8

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/franka_controller/franka_z_teach_point3_step_0_5mm.py
python3 src/franka_controller/franka_shear_test_points_3_8_fixed_1mm.py
```
