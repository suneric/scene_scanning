# scene_scanning
3D reconstruction of large scene using robot and rgb-d camera

## Dependencies
- ubuntu 20.04

- [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

- pcl 1.9
```
sudo apt-get install libpcl-dev python3-pcl
```

- opencv
```
sudo apt-get install libopencv-dev
pip install opencv-python=4.2.0.32
```

- [realsense sdk](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) and [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

- gazebo ros packages
```
sudo apt-get install ros-noetic-gazebo-ros-pkgs
    ros-noetic-gazebo-ros-control
    ros-noetic-effort-controllers
    ros-noetic-joint-state-controller
    ros-noetic-controller-manager
    ros-noetic-geographic-msgs
    ros-noetic-sensor-msgs

```

- [hector_quadrotor](http://wiki.ros.org/hector_quadrotor) and its depend packages
  - [hector_gazebo](http://wiki.ros.org/hector_gazebo)
  - [hector_localization](http://wiki.ros.org/hector_localization)
  - [hector_sensor_description](http://wiki.ros.org/hector_sensors_description)

## run
```
roslaunch ss_gazebo single_scanning.launch
```
