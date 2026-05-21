# Robot Connection

## 1. Physical setup

- Connect Ethernet directly from the Franka **Control unit** (the black box) to your PC.
- Unlock the brakes on Desk before any motion.
- Press the FCI (Franka Control Interface) **Activate** button on Desk.

## 2. Network configuration

Set your wired connection to **Manual / Static**:

| Field | Value |
|-------|-------|
| Address | `192.168.2.x` (any free IP on the same subnet as the robot) |
| Netmask | `255.255.255.0` |
| Gateway | (empty) |

Robot IP: **`192.168.2.10`**. Open `http://192.168.2.10` in a browser to access Desk.

The Desk username/password are set during initial robot setup by your lab admin. If you don't know them, ask the person who configured the robot — they are not shipped with this repository.

## 3. Verify connectivity

```bash
ping -c 3 192.168.2.10
```

## 4. Tell pyfranka the IP

Per shell (recommended for tests):

```bash
export ROBOT_IP=192.168.2.10
```

For MagTec workflows, also edit `magtec_models/config/hardware.yaml`:

```yaml
robot:
  ROBOT_IP: "192.168.2.10"
```

## 5. FCI status

The robot must show **FCI active** + brakes **open** before any move command. If Desk shows reflexes (orange/red LEDs on the robot), reset them from Desk before running any script.

## 6. Quick safety check (no MagTec)

```bash
conda activate franka_interface
cd ~/franka_cartesian_controller
python scripts/test_robot.py
```

This connects, prints current joint angles and end-effector pose, and disconnects without motion. If it works, your setup is good.

MagTec users can alternatively run:

```bash
cd ~/franka_cartesian_controller/magtec_models
python3 src/franka_controller/get_current_joints.py
```
