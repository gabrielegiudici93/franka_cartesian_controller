# MagTec Workflow Overview

## Typical pipeline

1. **Visualize** — confirm sensors and calibration (`examples/01` or `02`)
2. **No-touch baseline** — optional HDF5 without robot (`examples/03`)
3. **Collect** — normal and/or shear presses (`examples/05`, `06`, `07`)
4. **Train** — sklearn models on HDF5 (`examples/08`)
5. **Validate** — live GUI or predictor (`01`, `10_points_real_time_predictor`)

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
