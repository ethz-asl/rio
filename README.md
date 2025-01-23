# rio
Graph-based, sparse radar-inertial odometry m-estimation with barometer support and zero-velocity tracking.

**Paper**: https://arxiv.org/pdf/2408.05764
```
@inproceedings{girod2024brio,
author = {Rik Girod and Marco Hauswirth and Patrick Pfreundschuh and Mariano Biasio and Roland Siegwart},
title = {A robust baro-radar-inertial odometry m-estimator for multicopter navigation in cities and forests},
booktitle={IEEE Int. Conf. Multisensor Fusion Integration Intell. Syst.},
year={2024}
}
```

# Installation
Install [ROS noetic](https://wiki.ros.org/noetic/Installation/Ubuntu).
```
sudo apt install git build-essential python3-rosdep python3-catkin-tools ros-noetic-rqt-multiplot -y
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin init
cd src
git clone https://github.com/ethz-asl/rio.git
git clone https://github.com/ethz-asl/lpp.git
git clone https://github.com/rikba/gtsam_catkin.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

# Launch
Start RIO in one terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch rio rio.launch visualize:=true
```

Download an example dataset sequence.

Replay the bag
```
rosbag play 01_urban_night_H_raw.bag
```

# Supported sensors
| Sensor       | Default topic          | Message type              | Required | Note                              |
| ------------ | ---------------------- | ------------------------- | -------- | --------------------------------- |
| IMU          | /imu/data_raw          | sensor_msgs/Imu           | Yes      | Calibrate gyro turn-on-bias!      |
| IMU filtered | /imu/data              | sensor_msgs/Imu           | Yes      | for initialization                |
| Radar        | /radar/cfar_detections | sensor_msgs/PointCloud2   | Yes      |                                   |
| Barometer    | /baro/pressure         | sensor_msgs/FluidPressure | No       | Activate in [cfg](./cfg/rio.yaml) |

Radar point cloud format, see also [mav_sensors_ros](https://github.com/ethz-asl/mav_sensors_ros/blob/main/src/radar.cpp#L146-L236).
```
| Field name | Size    |
| ---------- | ------- |
| x          | FLOAT32 |
| y          | FLOAT32 |
| z          | FLOAT32 |
| doppler    | FLOAT32 |
| snr        | INT16   |
| noise      | INT16   |
```

# Dataset
The [dataset](https://doi.org/10.3929/ethz-b-000713420) contains 15 sequences of urban night, forest path, field, and deep forest handheld and flown data. It can be downloaded with and without camera images.

[Video of closed-loop quadrotor control in city and forest.](https://github.com/ethz-asl/rio/assets/11293852/7bc95fa8-6fa1-4172-ad63-5ae1d3a38d58)

| Scenario    | No. | Link w/o camera | Link with camera |
| ----------- | --- | --------------- | ---------------- |
| Urban Night | 01  | [01_urban_night_H_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=01_urban_night_H_raw_no_img.bag) | [01_urban_night_H_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=01_urban_night_H_raw.bag) |
|             | 02  | [02_urban_night_H_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=02_urban_night_H_raw_no_img.bag) | [02_urban_night_H_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=02_urban_night_H_raw.bag) |
|             | 03  | [03_urban_night_H_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=03_urban_night_H_raw_no_img.bag) | [03_urban_night_H_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=03_urban_night_H_raw.bag) |
|             | 04  | [04_urban_night_H_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=04_urban_night_H_raw_no_img.bag) | [04_urban_night_H_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=04_urban_night_H_raw.bag) |
|             | 05  | [05_urban_night_F_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=05_urban_night_F_raw_no_img.bag) | [05_urban_night_F_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=05_urban_night_F_raw.bag) |
|             | 06  | [06_urban_night_F_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=06_urban_night_F_raw_no_img.bag) | [06_urban_night_F_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=06_urban_night_F_raw.bag) |
|             | 07  | [07_urban_night_F_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=07_urban_night_F_raw_no_img.bag) | [07_urban_night_F_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=07_urban_night_F_raw.bag) |
| Forest Path | 08  | [08_forest_path_H_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=08_forest_path_H_raw_no_img.bag) | [08_forest_path_H_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=08_forest_path_H_raw.bag) |
|             | 09  | [09_forest_path_H_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=09_forest_path_H_raw_no_img.bag) | [09_forest_path_H_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=09_forest_path_H_raw.bag) |
|             | 10  | [10_forest_path_H_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=10_forest_path_H_raw_no_img.bag) | [10_forest_path_H_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=10_forest_path_H_raw.bag) |
|             | 11  | [11_forest_path_F_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=11_forest_path_F_raw_no_img.bag) | [11_forest_path_F_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=11_forest_path_F_raw.bag) |
|             | 12  | [12_forest_path_F_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=12_forest_path_F_raw_no_img.bag) | [12_forest_path_F_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=12_forest_path_F_raw.bag) |
| Flat Field  | 13  | [13_flat_field_F_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=13_flat_field_F_raw_no_img.bag) | [13_flat_field_F_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=13_flat_field_F_raw.bag) |
|             | 14  | [14_flat_field_F_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=14_flat_field_F_raw_no_img.bag) | [14_flat_field_F_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=14_flat_field_F_raw.bag) |
| Tree Slalom | 15  | [15_tree_slalom_F_raw_no_img.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=15_tree_slalom_F_raw_no_img.bag) | [15_tree_slalom_F_raw.bag](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=15_tree_slalom_F_raw.bag) |
|  |  |   | 
| All         | 01-15 | [raw_no_img_01-15.zip (200MB)](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download?path=%2F&files=raw_no_img_01-15.zip)  | [all](https://libdrive.ethz.ch/index.php/s/wfmZSbsx4x7yFPA/download) |

## Calibration
```
| Field name | IMU to radar | radar to cam |
| ---------- | ------------ | ------------ |
| x          | 0.122        | -0.020       |
| y          | 0.000        | -0.015       |
| z          | -0.025       | 0.000        |
| qx         | 0.67620958   | 0.000        |
| qy         | 0.67620958   | 0.7071068    |
| qz         | -0.20673802  | 0.7071068    |
| qw         | -0.20673802  | 0.000        |
```

```
<node name="imu_to_radar_broadcaster" pkg="tf2_ros" type="static_transform_publisher" args="0.122 0.000 -0.025 0.67620958 0.67620958 -0.20673802 -0.20673802 'bmi088' 'awr1843aop'" />
<node name="radar_to_cam_broadcaster" pkg="tf2_ros" type="static_transform_publisher" args="-0.020 -0.015 0.0 0.0 0.7071068 0.7071068 0.0 'awr1843aop' 'cam'" />
```

# Calibration Optimization (Experimental)
[rio_calibration_node.cpp](https://github.com/ethz-asl/rio/blob/main/src/rio_calibration_node.cpp) allows you to calibrate the rigid transformation between IMU and radar.

The calibration procedure:
1. Move the device in a loop, recording RIO odometry output (`/rio/odometry_optimize`), IMU (`/imu/data_raw`) and radar (`/radar/cfar_detections`). The device has to be located in the same position and orientation at start and goal to make the [loop closure](https://github.com/ethz-asl/rio/blob/main/cfg/calibration.yaml#L10-L12).
2. Set the initial guess in [calibration.yaml](https://github.com/ethz-asl/rio/blob/main/cfg/calibration.yaml#L6-L7).
3. `roslaunch rio calibration.launch`

This should refine your initial guess. Note you need to excite roll and pitch in your dataset. I noticed that you could also relax the [gyro bias process noise](https://github.com/ethz-asl/rio/blob/main/cfg/rio.yaml#L8), as it is well observable through loop closure.

# Related packages

| Package         | Description                     | Link                                                           |
| --------------- | ------------------------------- | -------------------------------------------------------------- |
| mav_sensors     | Linux user space sensor drivers | [mav_sensors](https://github.com/ethz-asl/mav_sensors)         |
| mav_sensors_ros | ROS sensor interface            | [mav_sensors_ros](https://github.com/ethz-asl/mav_sensors_ros) |

