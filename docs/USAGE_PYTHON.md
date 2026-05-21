# Using pyfranka_interface in Python

The library exposes a `Robot_` class with high-level helpers and callback-based control.

## Minimal connect + read state

```python
import pyfranka_interface as franka

robot = franka.Robot_(
    "192.168.2.10",      # robot IP
    False,                # use_gripper_hand (set True if Panda has hand)
    hand_franka=False,
    auto_init=True,
    speed_factor=0.1,
)

state = robot.getState()
print("joints:", state.q)
print("EE pose (4x4):\n", state.T)
```

## Absolute Cartesian move

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

target = np.eye(4)
target[:3, :3] = R.from_euler("xyz", [180, 0, 0], degrees=True).as_matrix()
target[:3, 3] = [0.500, 0.420, 0.034]

robot.move("absolute", target, 2.0)   # 2-second motion
```

## Relative move (delta in tool frame)

```python
delta = np.eye(4)
delta[:3, 3] = [0.0, 0.0, -0.001]    # 1 mm down
robot.move("relative", delta, 0.5)
```

## Joint move

```python
target_joints = [-1.46, -1.44, 1.85, -1.68, 1.46, 1.86, 0.86]
robot.move_joints(target_joints, 0.05)   # speed_factor 5%
```

## Custom control callback

```python
def control_callback(state, init_state, cur_time):
    cmd = franka.Robot_.ReturnControlCommand()
    # fill cmd.joint_space_command or cmd.cartesian_command
    if cur_time > 5000:                  # ms
        cmd.running_controller = False
    return cmd

robot.set_controller(control_callback, "cartesian_pose")
robot.run()
```

Modalities: `joint_pos`, `joint_vel`, `torque`, `cartesian_pose`.

## Examples

Working scripts live in:

```
pyfranka_interface/src/examples/
  ├── example.py
  ├── example_embedded_controller.py
  ├── test_FL_franka_interface.py
  └── trajectory.py
```

Run them with the conda env activated and `LD_LIBRARY_PATH` set.

## Higher-level lab code

For full tactile-press/shear pipelines see `magtec_models/src/franka_controller/`.
