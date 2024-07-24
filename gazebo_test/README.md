# Gazebo Test
A simple setup and workflow test in Gazebo

Requirements
1. Ubuntu 20.04
2. ROS Noetic
3. Packages
   - apriltag_ros
   - dingo_simulator
   - dingo_desktop
  
Gazebo setup
1. Custom April Tag Object resized to 0.5mx0.5m: https://github.com/koide3/gazebo_apriltag/tree/master/models/Apriltag36_11_00000
2. Custom config file for Dingo, see variables below

Instructions
1. Launch the tag detector and pose estimator with $ roslaunch gazebo_test gazebo_continuous_detector.launch
2. Run the test docking script with $ rosrun gazebo_test gazebo_docking_node.py


Initial Gazebo Test
![](https://github.com/nvanheyst/dingo_april_tag_docking/blob/master/gazebo_test/media/poc.gif)

Config for test Dingo:

DINGO_OMNI=0

DINGO_LASER=1
DINGO_LASER_MODEL='ust10'
DINGO_LASER_TOPIC='front/scan'

DINGO_LASER_SECONDARY=1
DINGO_LASER_SECONDARY_MODEL='ust10'
DINGO_LASER_SECONDARY_TOPIC='rear/scan'

DINGO_IMU_MICROSTRAIN=1
DINGO_IMU_MICROSTRAIN_NAME='microstrain'
DINGO_IMU_MICROSTRAIN_PORT='/dev/microstrain'

export DINGO_REALSENSE=1
export DINGO_REALSENSE_MODEL='d435'
export DINGO_REALSENSE_MOUNT='front'
export DINGO_REALSENSE_TOPIC='realsense'
export DINGO_REALSENSE_OFFSET='-0.2 0 0.2'
export DINGO_REALSENSE_RPY='0 0 0'
