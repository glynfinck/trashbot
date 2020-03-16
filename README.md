# YOLO ROS: Real-Time Object Detection for ROS

## Overview

This is a ROS package developed for object detection in camera images. You only look once (YOLO) is a state-of-the-art, real-time object detection system. In the following ROS package you are able to use YOLO (V3) on GPU and CPU. The pre-trained model of the convolutional neural network is able to detect pre-trained classes including the data set from VOC and COCO, or you can also create a network with your own detection objects. For more information about YOLO, Darknet, available training data and training YOLO see the following link: [YOLO: Real-Time Object Detection](http://pjreddie.com/darknet/yolo/).

The YOLO packages have been tested under ROS Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author: [Marko Bjelonic](https://www.markobjelonic.com), marko.bjelonic@mavt.ethz.ch**

**Affiliation: [Robotic Systems Lab](http://www.rsl.ethz.ch/), ETH Zurich**

![Darknet Ros example: Detection image](darknet_ros/doc/test_detection.png)
![Darknet Ros example: Detection image](darknet_ros/doc/test_detection_anymal.png)

Based on the [Pascal VOC](https://pjreddie.com/projects/pascal-voc-dataset-mirror/) 2012 dataset, YOLO can detect the 20 Pascal object classes:

- person
- bird, cat, cow, dog, horse, sheep
- aeroplane, bicycle, boat, bus, car, motorbike, train
- bottle, chair, dining table, potted plant, sofa, tv/monitor

Based on the [COCO](http://cocodataset.org/#home) dataset, YOLO can detect the 80 COCO object classes:

- person
- bicycle, car, motorbike, aeroplane, bus, train, truck, boat
- traffic light, fire hydrant, stop sign, parking meter, bench
- cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe
- backpack, umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket
- bottle, wine glass, cup, fork, knife, spoon, bowl
- banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake
- chair, sofa, pottedplant, bed, diningtable, toilet, tvmonitor, laptop, mouse, remote, keyboard, cell phone, microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, teddy bear, hair drier, toothbrush

## Citing

The YOLO methods used in this software are described in the paper: [You Only Look Once: Unified, Real-Time Object Detection](https://arxiv.org/abs/1506.02640).

If you are using YOLO V3 for ROS, please add the following citation to your publication:

M. Bjelonic
**"YOLO ROS: Real-Time Object Detection for ROS"**,
URL: https://github.com/leggedrobotics/darknet_ros, 2018.

    @misc{bjelonicYolo2018,
      author = {Marko Bjelonic},
      title = {{YOLO ROS}: Real-Time Object Detection for {ROS}},
      howpublished = {\url{https://github.com/leggedrobotics/darknet_ros}},
      year = {2016--2018},
    }

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
