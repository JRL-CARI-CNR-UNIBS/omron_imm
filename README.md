# Omron IMM

This repository provides tools and launch files for connecting and operating an **Omron mobile manipulator** with optional Robotiq gripper and force/torque (FT) sensor support.  

---

## 🔑 SSH Connection

To connect to the mobile robot via SSH:

```bash
ssh omron@192.168.1.77
```

if you have some errors try:
```bash
ssh -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa omron@192.168.1.77
```



## 🚀 Launching on the Mobile Robot

Once connected to the robot via SSH, you can bring up the system using ROS 2 launch files.

### Basic bringup with Robotiq gripper (fake mode, no RViz)
```bash
ros2 launch omron_imm_bringup bringup.launch.py \
    include_robotiq_gripper:=true \
    use_fake_gripper:=true \
    rviz:=false
```
### Bringup with Robotiq gripper and FT sensor
```bash
ros2 launch omron_imm_bringup bringup.launch.py \
    include_robotiq_gripper:=true \
    use_fake_gripper:=true \
    rviz:=false \
    include_robotiq_ft_sensor:=true
```
## ⏱️ Time Synchronization

Synchronize time with your NTP server:

```bash
sudo ntpdate -u 192.168.1.2
```

## 📡 Force/Torque (FT) Sensor on Mobile Robot

To start the Robotiq FT300s sensor node:

```bash
ros2 launch robotiq_ft_sensor_hardware ft_sensor_standalone.launch.py \
    namespace:=omron \
    frame_id:=omron/ft300s_sensor
```
## 💻 Local Launches

When working locally (e.g., on your workstation):

### Navigation (Nav2)

```bash
ros2 launch lampo_demo omron_nav.launch.py
```

### RVIZ2
```
ros2 launch omron_imm_bringup remote_rviz.launch.py
```