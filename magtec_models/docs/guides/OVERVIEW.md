# MagTec Workflow Overview

**New dataset:** [Data Collection Guide](DATA_COLLECTION.md).  
**Manual robot control:** [Teleoperation Guide](TELEOPERATION.md).

## Typical pipeline

1. **Visualize** — confirm sensors and calibration (`examples/01` or `02`)
2. **No-touch baseline** — optional HDF5 without robot (`examples/03`)
3. **Collect** — normal and/or shear presses (`examples/05`, `06`, `07`)
4. **Train** — sklearn models on HDF5 (`examples/08`)
5. **Validate** — live GUI or predictor (`01`, `10_points_real_time_predictor`)

## Collection engine

All robot + skin HDF5 collection flows through **`src/franka_controller/franka_skin_test.py`**:

- **Example 05** → `franka_skin_test_multiple_points.py` → calls the engine
- **Shear examples** → `franka_skin_test_shear_forces.py` (and thin wrappers) → same sensor stack and HDF5 structure
- **Example 03** → `collect_no_touch_data.py` (skin only, no robot)
- **Example 04** → robot + FT validation only (no engine, no skin serial)

See [README.md](../../README.md#data-collection-franka_skin_testpy-core-engine) for the full script map.

## Hardware dependencies

| Script type | Robot | FT sensor | Magnetic skin |
|-------------|-------|-----------|---------------|
| Visualization | No | Optional | Yes |
| No-touch collect | No | No | Yes |
| 10-point press | Yes | Yes | No |
| Multi-point / shear collect | Yes | Yes | Yes |
| Training | No | No | No (reads HDF5) |

## Port conflicts

If visualization shows `SerialException` or stuck calibration:

```bash
jobs -l          # kill stopped Python jobs using the port
kill %1          # example
```

Run **either** collection **or** visualization on the magnetic port, not both.
