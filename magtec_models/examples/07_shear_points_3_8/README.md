# Example 07 — Shear on Taxels 3–8 (Fixed Displacement)

For the soft-silicone 6-taxel experiment region (points 3,4,5,6,7,8).

- XY shear: 1 mm per direction
- Z approach: 3.75 mm (2.75 mm touch + 1 mm press) — edit in script if needed
- Manual pause before points 3, 5, 7
- No magnetic serial (robot + FT only)

## Z teaching (calibrate approach)

```bash
python3 src/franka_controller/franka_z_teach_point3_step_0_5mm.py
```

Then update `Z_APPROACH_M` in `franka_shear_test_points_3_8_fixed_1mm.py`.

## Run experiment

```bash
./examples/07_shear_points_3_8/run.sh
```
