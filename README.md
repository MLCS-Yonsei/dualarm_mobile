# Dualarm_mobile
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

Slam
```bash
roslaunch dualarm_mobile_slam_navi slam.launch
```

Navi
```bash
roslaunch dualarm_mobile_slam_navi navi.launch
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
rosrun map_server map_saver -f /home/seungchul/catkin_ws/src/dualarm_mobile/dualarm_mobile_slam_navi/maps/map
```

Goal pub
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseSmped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.1, y: 7.9, z: 0.0}, orientation: {z: -0.1, w: 1.0}}}'
```
