# Misty West - Trashbot

## Overview



**Authors: Glyn Finck, Keenan McConkey, Masayoshi Kuwabara, Austin Khorram**

**Affiliation: [Misty West](https://www.mistywest.com/), [UBC Vancouver](https://www.ubc.ca/)**

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionally,thefollowing software is required:

- [OpenCV](http://opencv.org/) (computer vision library),
- [Realsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) (Intel Realsense camera SDK)
- [boost](http://www.boost.org/) (c++ library),

### Building

#### Realsense

In order to install trashbot, start by cloning the latest verion of realsense ros using the instructions found at [jetson hacks](https://www.jetsonhacks.com/2019/10/25/realsense-ros-wrapper-jetson-nano/) into your catkin workspace.

Next, check that your realsense camera is connected and run the following commands from your catkin workspace directory.

    source devel/setup.bash
    roslaunch realsense2_camera rs_camera.launch

#### Darknet ROS

The next package to install will be [Darknet ROS](https://github.com/leggedrobotics/darknet_ros) and can be installed and built using the installation instructions at the provided link the github repository.

## Basic Usage

In order to get YOLO ROS: Real-Time Object Detection for ROS to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files that you need to change from the `darknet_ros` package. These are specifically the parameter files in `config` and the launch file from the `launch` folder.

## Nodes

### Node: object_tracker

### Node: motor_intermediate

### Node: trashbot_2dnav

### Node: trashbot_ai

This is the main trashbot node for all of the internal logic required for finding, navigating, picking up, dropping off bottles within a space.

### ROS related parameters

#### Subscribed Topics

* **`object_tracker/bounding_boxes`** ([darknet_ros_msgs::BoundingBoxes])

    Publishes an array of bounding boxes that gives information of the position and size of the bounding box in pixel coordinates.


#### Published Topics

* **`robot_state`** ([std_msgs::Int8])

    The current robot state represented as an integer with the following encoding: STATE_STOP = 0, STATE_SET_DROPOFF = 1, STATE_FIND_BOTTLE = 2, STATE_NAV_BOTTLE = 3, STATE_PICKUP_BOTTLE = 4, STATE_NAV_DROPOFF = 5, STATE_DROPOFF_BOTTLE = 6.

* **`intermediate_vel`** ([geometry_msgs::Twist])

    This expresses velocity in free space broken into its linear and angular parts.

* **`servo1`** ([std_msgs::UInt16])

    The angle to set the 'arm' servo on the robot to.


* **`servo2`** ([std_msgs::UInt16])

    The angle to set the 'claw' servo on the robot to.

## Trashbot Demo Usage

### Connecting to Trashbot via SSH
