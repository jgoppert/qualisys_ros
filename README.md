# qualisys_ros
ROS Qualisys Package

## Installation

This is a standard ROS1 packages. We plan to support ROS2 by end of Q1 2021.


```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:jgoppert/qualisys_ros
cd ~/catkin_ws
catkin init
catkin build
```

## Usage

### Live

The first launch script starts a vehicle for live testing which routes motion capture packets via mavros to a vehicle for navigtional or sensor emulation.

```bash
. ./devel/setup.bash
roslaunch qualisys_ros px4_live.launch
```

### Simulation

The second launch script starts a simulated world in gazebo which contains a virtual cameras this is positioned using Qualisys mocap data. There is a bag file of qualisys data for offline testing int this repository.

```bash
. ./devel/setup.bash
roslaunch qualisys_ros abu_dhabi.launch
```
