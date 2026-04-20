# SSH TestBed

## install
``` sh
cd ~/{your_workspace}/src
unzip fms_custom_tb.zip
cd ..
colcon build
```
``` sh
source ~/.bashrc
```

## simulation ssh_tb

### launch ssh_tb Gazebo with Turtlebot3
``` sh
export TURTLEBOT3_MODEL=waffle
ros2 launch fms_custom_tb fms_ssh_tb.launch.py
```
### launch Turtlebot3 SLAM
``` sh
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
### Keyboard Teleop
``` sh
ros2 run turtlebot3_teleop teleop_keyboard
```

### save map
``` sh
ros2 run nav2_map_server map_saver_cli -f ~/map
```
