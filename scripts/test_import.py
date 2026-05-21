#!/usr/bin/env python3
"""Software-only install check (no robot required)."""
import sys

def main() -> int:
    print("=" * 60)
    print("TEST 1 — import pyfranka_interface")
    print("=" * 60)
    try:
        import pyfranka_interface as franka
    except ImportError as e:
        print("FAIL:", e)
        print("Hint: conda activate franka_interface")
        print("Hint: export LD_LIBRARY_PATH=$HOME/franka_cartesian_controller/pyfranka_interface/third_party/libfranka/lib:$LD_LIBRARY_PATH")
        return 1

    print("OK — module loaded")
    print("  Robot_ class:", hasattr(franka, "Robot_"))
    print("  Python:", sys.version.split()[0])
    print()
    print("Software install looks good. Run scripts/test_robot.py when the robot is on the network.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
