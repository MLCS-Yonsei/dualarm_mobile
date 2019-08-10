# dualarm_mobile
ROS SLAM &amp; Navigation code for mechanum wheeled omnidirectional mobile platform

## Dependency
Dependency from apt:
```bash
sudo apt-get install ros-kinetic-robot-gmapping
sudo apt-get install ros-kinetic-robot-amcl
sudo apt-get install ros-kinetic-joy
```
Dependency repository:
https://github.com/MLCS-Yonsei/teb_local_planner.git

## Installation
```bash
cd catkin_ws/src
git clone https://github.com/MLCS-Yonsei/dualarm_mobile.git
cd ..
catkin_make
source ~/.bashrc && source ~/catkin_ws/devel/setup.bash
```

## Usage

### Running an environment
Bringup
```bash
roslaunch dualarm_mobile_bringup robot.launch
```

SLAM-gmapping
```bash
roslaunch dualarm_mobile_navigation gmapping.launch
```

AMCL
```bash
roslaunch dualarm_mobile_navigation amcl.launch
```

teb_local_planner
```bash
roslaunch dualarm_mobile_navigation teb_local_planner.launch
```

Teleop (keyboard)
```bash
roslaunch dualarm_mobile_teleop dualarm_mobile_teleop_key.launch
```
Teleop (xbox360 controller)
```bash
roslaunch dualarm_mobile_teleop dualarm_mobile_teleop_joy.launch
```

Map saver
```bash
rosrun map_server map_saver -f $(find dualarm_mobile_navigation)/maps/map
```

Goal pub
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseSmped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.1, y: 7.9, z: 0.0}, orientation: {z: -0.1, w: 1.0}}}'
```
