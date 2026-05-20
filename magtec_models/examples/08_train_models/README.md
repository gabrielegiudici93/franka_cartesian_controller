# Example 08 — Train Models

Train location / stretch / force regressors from collected HDF5 data.

## Prerequisites

- Normal-force dataset folder, e.g. `data/Multiple_Points/my_run_000pct.h5` parent dir
- Optional shear dataset for combined training

## Run (template)

From the repo root, after activating the env:

```bash
cd magtec_models
./examples/08_train_models/run.sh
```

Edit `run.sh` and set:

- `NORMAL_DIR` — folder with normal-force HDF5 files
- `SHEAR_DIR` — folder with shear HDF5 (or leave empty)
- `RUN_LABEL` — output name under `models/`

## Inspect data first

```bash
python3 src/training/inspect_h5_files.py data/Multiple_Points/<your_run>/
```

## Clean sequences (optional)

```bash
python3 src/training/clean_sequences.py --help
```
