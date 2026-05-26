# Example 10 — Keyboard Teleoperation (Robot + Sensors)

Manual Cartesian control of the Franka from the terminal, with optional live FT + magnetic skin GUI.

**Full guide:** [docs/guides/TELEOPERATION.md](../../docs/guides/TELEOPERATION.md)

## Requirements

- Robot unlocked, FCI active — [../../../docs/ROBOT_CONNECTION.md](../../../docs/ROBOT_CONNECTION.md)
- `config/hardware.yaml` configured
- Terminal with raw keyboard input (run in a real TTY, not inside a non-interactive job)
- Only one process on the magnetic serial port

## Run

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
cd ~/franka_cartesian_controller/magtec_models

python3 src/franka_controller/teleop_franka_keyboard.py
```

## Configure

Edit the top of `src/franka_controller/teleop_franka_keyboard.py`:

- `SELECTED_POSITION_ID` — grid point to move to before teleop (default `32`)
- `SELECTED_OFFSET` — `'center'`, `'n'`, `'s'`, etc.
- `ENABLE_GUI` — `True` for live sensor plots during teleop

## Controls (summary)

| Key | Action |
|-----|--------|
| `8` / `2` | Move ±X |
| `4` / `6` | Move ±Y |
| `-` / `+` | Move ±Z |
| `*` / `/` | Increase / decrease step size |
| `p` | Print current pose |
| `q` or `c` | Quit |
