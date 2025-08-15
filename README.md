# Floor-Based Obstacle Detection for ROS2 Navigation

A ROS2 package that detects floor-level obstacles using Intel RealSense depth cameras for autonomous robot navigation. This package separates floor points from obstacle points based on camera height and publishes filtered point clouds for Nav2 integration.

## Features

- **Height-based point cloud filtering** using camera extrinsic parameters
- **Dual point cloud publishing** for floor and obstacle visualization
- **Configurable parameters** for different camera heights and orientations

## Use Cases

- Autonomous mobile robots in indoor environments
- Service robots navigating around furniture
- Any robot needing to detect obstacles below traditional 2D LiDAR height

## Hardware Requirements

- Intel RealSense depth camera (D455, D435, etc.)
- ROS2 Humble or later
- Compatible with Jetson devices and Intel NUC

## Key Parameters

- Camera height: 50cm (configurable)
- Detection range: 0.3m - 4.0m
- Floor detection: 35-65cm below camera
- Obstacle detection: 10cm above to 25cm below camera level

## Topics

**Published:**
- `/obstacles/pointcloud` - Filtered obstacle points for Nav2
- `/floor/pointcloud` - Floor points for visualization

**Subscribed:**
- `/camera/camera/depth/image_rect_raw` - RealSense depth image
- `/camera/camera/depth/camera_info` - Camera intrinsic parameters

