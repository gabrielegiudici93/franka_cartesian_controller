# Robot Connection

## 1. Physical setup

- Connect Ethernet directly from the Franka **Control unit** (the black box) to your PC.
- Unlock the brakes on Desk before any motion.
- Press the FCI (Franka Control Interface) **Activate** button on Desk.

## 2. Network configuration

Set your wired connection to **Manual / Static**:

| Field | Value |
|-------|-------|
| Address | `172.16.0.10` (or any free IP on the same subnet) |
| Netmask | `255.255.255.0` |
| Gateway | (empty) |

Common robot IP: `172.16.0.2`. Open `http://172.16.0.2` in a browser to access Desk.

The Desk username/password are set during initial robot setup by your lab admin. If you don't know them, ask the person who configured the robot — they are not shipped with this repository.

## 3. Verify connectivity

```bash
ping -c 3 172.16.0.2
```

## 4. Tell pyfranka the IP

Edit `magtec_models/config/hardware.yaml`:

```yaml
robot:
  ROBOT_IP: "172.16.0.2"
```

Or export per-shell:

```bash
export robot_ip=172.16.0.2
```

## 5. FCI status

The robot must show **FCI active** + brakes **open** before any move command. If Desk shows reflexes (orange/red LEDs on the robot), reset them from Desk before running any script.

## 6. Quick safety check

```bash
cd ~/franka_cartesian_controller/magtec_models
python3 src/franka_controller/get_current_joints.py
```

This connects, prints current joint angles, and disconnects. If it works, your setup is good.
