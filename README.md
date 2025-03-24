# Omron imm

Ssh connection:
```bash
ssh -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa omron@192.168.1.77
```


On the mobile robot (remotly via ssh):
```bash
    ros2 launch omron_imm_bringup bringup.launch.py include_robotiq_gripper:=true use_fake_gripper:=true rviz:=false   
```

Locally
- Nav 2 
```bash
ros2 launch lampo_demo omron_nav.launch.py
```
- Rviz:

```bash
ros2 launch omron_imm_bringup remote_rviz.launch.py
```