# ESKF_LIO

Loosely Coupled Lidar Inertial Odometry based Error State Kalman Filter.

## How to use

### Install

```bash
mkdir -p ~/lio_ws/src && cd ~/lio_ws/src
git clone https://github.com/koide3/fast_gicp.git
git clone https://github.com/RyuYamamoto/eskf_lio.git
cd ../
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run

```bash
source ~/lio_ws/install/setup.bash
ros2 launch eskf_lio eskf_lio.launch.xml
```
