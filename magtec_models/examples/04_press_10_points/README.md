# Example 04 — Press 10 Points (FT Only)

Robot moves to 10 predefined XY offsets and presses to ~3N using the FT sensor only.
**Does not** open the magnetic skin port (safe to run alongside visualization if needed).

## Run

From the repo root, with the env active and the robot reachable (see [`docs/ROBOT_CONNECTION.md`](../../../docs/ROBOT_CONNECTION.md)):

```bash
cd magtec_models
./examples/04_press_10_points/run.sh
```

Press Enter between points. Loops until Ctrl+C.

## When to use

- Validate robot + FT before full skin collection
- Quick mechanical check of the 10-point grid
