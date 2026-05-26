"""
Backward-compatible import path for data-collection scripts.

Older code imports ``validation_tests.real_time_predictor``; the implementation
lives in ``10_points_real_time_predictor.py``.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

_IMPL = Path(__file__).resolve().parent / "10_points_real_time_predictor.py"
_spec = importlib.util.spec_from_file_location("_points_real_time_predictor_impl", _IMPL)
if _spec is None or _spec.loader is None:
    raise ImportError(f"Cannot load predictor implementation from {_IMPL}")
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

RealTimePredictorGUI = _mod.RealTimePredictorGUI
ModelPredictor = _mod.ModelPredictor

__all__ = ["RealTimePredictorGUI", "ModelPredictor"]
