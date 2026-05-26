# Publish to GitHub

## Existing repository (daily use)

See **[GIT_WORKFLOW.md](GIT_WORKFLOW.md)** for `pull`, `commit`, and `push` commands in English.

Quick version:

```bash
cd ~/franka_cartesian_controller
git pull --rebase origin main
git add .
git commit -m "Describe your change"
git pull --rebase origin main
git push origin main
```

---

## First-time setup (new empty repo on GitHub)

```bash
cd ~/franka_cartesian_controller
git init
git lfs install   # optional, for .h5 / .joblib if you ship demo data

git add .
git commit -m "Initial release: Franka Cartesian Controller + MagTec skin"
git remote add origin git@github.com:gabrielegiudici93/franka_cartesian_controller.git
git branch -M main
git push -u origin main
```

Use SSH (`git@github.com:...`) if possible; otherwise HTTPS with a Personal Access Token.

---

## What to keep out

The top-level `.gitignore` already excludes:

- `magtec_models/config/hardware.yaml` (per-machine)
- `magtec_models/data/**/*.h5`, `models/**`, `plots/**`, `logs/**`
- `pyfranka_interface/build/`, `*.so`, `*.egg-info/`, `__pycache__/`

---

## What is vendored (intentional)

- `pyfranka_interface/src/` — full controller source
- `pyfranka_interface/third_party/libfranka/` (v0.9.2 headers + `.so`)
- `pyfranka_interface/third_party/pybind11/`

Collaborators can clone and build without any external lab repo.

---

## Sample data (optional)

```bash
git lfs track "*.h5"
cp /path/to/small_run.h5 magtec_models/data/Multiple_Points/demo_run_000pct.h5
git add .gitattributes magtec_models/data/Multiple_Points/demo_run_000pct.h5
git commit -m "Add demo dataset"
git push origin main
```

Otherwise keep `data/` empty and document where to download runs.
