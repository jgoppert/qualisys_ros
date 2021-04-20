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

The first launch script starts a vehicle for live testing which routes motion capture packets via mavros to a vehicle for navigtion or sensor emulation.

```bash
. ./devel/setup.bash
roslaunch qualisys_ros px4_live.launch
```

### Simulation

The second launch script starts a simulated world in Gazebo which contains a virtual camera that is positioned using Qualisys mocap data. A bag file of Qualisys data for offline testing is included in this repository.

```bash
. ./devel/setup.bash
roslaunch qualisys_ros bag.launch
```
