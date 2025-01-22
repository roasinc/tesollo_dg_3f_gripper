# tesollo_dg_3f_gripper
ROS2 package for TESOLLO DG-3F Gripper with MODBUS-RTU

## Installation
```
shell
$ cd ~/dev_ws/src
$ git clone https://github.com/roasinc/tesollo_dg_3f_gripper.git
$ rosdep install --from-path tesollo_dg_3f_gripper --ignore-src -r -y
```

## Build
```
shell
$ cd ~/dev_ws
$ colcon build --symlink-install
```


## Execute
```
shell
$ ros2 launch delto_3f_description upload_gripper.launch.py
$ ros2 run delto_3f_driver main_node --ros-args -p port_name:=/dev/ttyUSB0 -r joint_states:=gripper_joint_states