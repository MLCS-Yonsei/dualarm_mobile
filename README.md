# Dualarm_mobile
SLAM &amp; Navigation code for mechanum-wheel omnidirectional mobile platform

## Dependency
Dependency from apt:
```bash
sudo apt-get install ros-kinetic-robot-gmapping
sudo apt-get install ros-kinetic-robot-amcl
sudo apt-get install ros-kinetic-teb-local-planner
```
Dependency repository:
https://github.com/MLCS-Yonsei/dualarm_mobile_dep.git

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

Slam (static environment)
```bash
roslaunch dualarm_mobile_slam_navi slam_st.launch
```

Navi (static environment)
```bash
roslaunch dualarm_mobile_slam_navi navi_st.launch
```

Slam (dynamic environment)
```bash
roslaunch dualarm_mobile_slam_navi slam_dy.launch
```

Navi (dynamic environment)
```bash
roslaunch dualarm_mobile_slam_navi navi_dy.launch
```

Teleop
```bash
roslaunch dualarm_mobile_teleop dualarm_mobile_teleop_key.launch
```

Map saver
```bash
rosrun map_server map_saver -f /home/seungchul/catkin_ws/src/dualarm_mobile/dualarm_mobile_slam_navi/maps/map
```

Goal pub
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseSmped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.1, y: 7.9, z: 0.0}, orientation: {z: -0.1, w: 1.0}}}'
```
