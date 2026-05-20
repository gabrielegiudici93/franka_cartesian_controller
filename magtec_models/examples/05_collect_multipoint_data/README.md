# Example 05 — Multi-Point Data Collection (Normal Forces)

Full collection: robot + FT + magnetic skin → HDF5 under `data/Multiple_Points/`.

## Before running

1. `config/hardware.yaml` configured
2. Skin calibrated in visualization example
3. Robot unlocked, workspace clear

## Run

```bash
cd ~/franka_magtec_workspace/magtec_models
./examples/05_collect_multipoint_data/run.sh
```

Follow terminal prompts for stretch levels (0%, 10%, 20% by default).

## Output

`data/Multiple_Points/<run_label>_000pct.h5`, etc.

Train with Example 08.
