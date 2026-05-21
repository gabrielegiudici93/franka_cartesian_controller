# Using franka_interface in C++

The C++ side lives in `pyfranka_interface/src/cartesian_franka/`. Headers are:

- `robot.hpp` / `robot.tpp` — main `RobotInterface<T>` template
- `joint_motion_generator.hpp`
- `cartesian_motion_generator.hpp`
- `torque_FL_motion_generator.hpp`
- `com_def.hpp` — types: `SimpleState`, `ReturnControlCommand`, ...

## Minimal example

```cpp
#include "cartesian_franka/robot.hpp"

using namespace franka_interface;

ReturnControlCommand control_callback(SimpleState& state,
                                      SimpleState& init_state,
                                      double cur_time) {
    ReturnControlCommand cmd;
    // ... fill cmd.cartesian_command or cmd.joint_space_command ...
    if (cur_time > 5000.0) cmd.running_controller = false;
    return cmd;
}

int main() {
    RobotInterface<void> bot("192.168.2.10");
    bot.setController(&control_callback, ControlMode::cartesian_pose);
    bot.run();   // blocking
}
```

## With persistent data

```cpp
struct MyData {
    int counter = 0;
    Eigen::Vector3d last_pos;
};

ReturnControlCommand callback(SimpleState& s, SimpleState& i, MyData& d, double t) { ... }

RobotInterface<MyData> bot("192.168.2.10");
bot.setCtrl(&callback, ControlMode::cartesian_pose);
bot.run();
```

## Single-shot motions (no callback)

```cpp
bot.translate({0.0, 0.0, -0.001}, 0.5);     // 1mm down in 0.5s
bot.move_joints({-1.46,-1.44,1.85,-1.68,1.46,1.86,0.86}, 1.0);
bot.move(target_pose, 2.0);                  // Eigen::Affine3d
bot.extMove("absolute", target_matrix, 2.0); // Eigen::Matrix4d
```

## Async control

Use `bot.runDetach()` to run the control loop in a background thread, then `bot.closeControlThread()` before exit.

## Building your own C++ project

Link against:

```
-I pyfranka_interface/third_party/libfranka/include
-I pyfranka_interface/src
-I /usr/include/eigen3
-L pyfranka_interface/third_party/libfranka/lib
-lfranka -lpthread
```

See `pyfranka_interface/setup.py` for the exact flags used to build the Python binding.

Full reference: [pyfranka_interface/README.md](../pyfranka_interface/README.md)
