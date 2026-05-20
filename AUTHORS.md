# Authors

This repository is maintained by:

- **Gabriele Giudici** — main author / maintainer  
  Workspace orchestration, documentation, `magtec_models` (tactile-skin data collection, real-time 15-taxel visualization, ML training pipelines), and current maintenance of the vendored `pyfranka_interface`.

- **Valerio Modugno** — co-author  
  Original developer of `pyfranka_interface` (Franka Cartesian Controller, Python + C++ bindings via pybind11). Vendored here under its original BSD 2-Clause license; see `pyfranka_interface/LICENSE`.

## Third-party

- `pyfranka_interface/third_party/libfranka/` — Franka Emika GmbH (Apache 2.0)
- `pyfranka_interface/third_party/pybind11/` — Wenzel Jakob et al. (BSD-style)
- `pyfranka_interface/waf` — ResiBots/waf authors (BSD)

If you build on this work, please cite both authors above and respect the original third-party licenses.
