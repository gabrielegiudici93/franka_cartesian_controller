# Installation — Franka Cartesian Controller

Step-by-step build and install. Tested on **Ubuntu 20.04 / 22.04 / 24.04**.

The main tutorial lives in the root [README.md](../README.md); this file is the detailed reference.

---

## 1. System prerequisites

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git \
    libpoco-dev libeigen3-dev \
    python3-dev
```

USB serial (optional, for force sensor / MagTec later):

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

---

## 2. Real-time kernel (recommended)

https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel

Control can work without it but may drop frames.

---

## 3. Conda / Mamba environment

Install [Miniforge](https://github.com/conda-forge/miniforge) if conda is not available, then:

```bash
cd ~/franka_cartesian_controller
mamba env create -f environment.yml
conda activate franka_interface
```

---

## 4. libfranka

Vendored **0.9.2** under:

```
pyfranka_interface/third_party/libfranka/
  ├── include/
  └── lib/
```

### Symlinks

The linker expects `libfranka.so`. If only `libfranka.so.0.9.2` is present:

```bash
cd ~/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib
ln -sf libfranka.so.0.9.2 libfranka.so
ln -sf libfranka.so.0.9.2 libfranka.so.0.9
```

### Runtime path

```bash
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH
```

Add that line to `~/.bashrc` for convenience.

---

## 5. Build and install `pyfranka_interface`

**Recommended (setup.py):**

```bash
conda activate franka_interface
export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH

cd ~/franka_cartesian_controller/pyfranka_interface
python setup.py build_ext --inplace
python setup.py install
```

**Alternative (waf)** — only if `wscript` exists in `pyfranka_interface/`:

```bash
cd ~/franka_cartesian_controller/pyfranka_interface
python3 ./waf configure --python
python3 ./waf
python3 setup.py install
```

---

## 6. Verify (no robot)

```bash
conda activate franka_interface
cd ~/franka_cartesian_controller
python scripts/test_import.py
```

---

## 7. Robot test (no MagTec)

Network and FCI: [ROBOT_CONNECTION.md](ROBOT_CONNECTION.md).

```bash
export ROBOT_IP=192.168.2.10   # your robot IP
python scripts/test_robot.py
```

---

## 8. MagTec skin stack (optional)

```bash
pip install -r requirements-magtec.txt
cd magtec_models
cp config/hardware.example.yaml config/hardware.yaml
# edit ROBOT_IP, ports, etc.
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ImportError: libfranka.so.0.9` | `libfranka.so` symlinks + `LD_LIBRARY_PATH` |
| `cannot find -lfranka` | Same symlinks in `third_party/libfranka/lib/` |
| `waf configure` / no `wscript` | Use `setup.py` build (section 5) |
| `waf configure` fails on pybind11 | `conda activate franka_interface`; `which python3` |
| Robot not reachable | [ROBOT_CONNECTION.md](ROBOT_CONNECTION.md) |
| `reflex_aborted` | Lower `speed_factor`; reset reflexes on Desk |
| Permission denied on `/dev/ttyUSB*` | `dialout` group + re-login |
