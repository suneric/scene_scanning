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
sudo apt-get install ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-effort-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-controller-manager \
    ros-noetic-geographic-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-unique-identifier \
    ros-noetic-geographic-info
```

- [hector_quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) and its depend packages
1. clone the repo to you workspace (e.g., 'catkin_ws/src')
```
git clone -b noetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
```
2. change the version of hector_quadrotor.rosinstall to noetic-devel
```
cd catkin_ws
rosdep install --from-paths src --ignore-src
```
3. build the catkin_ws
```
catkin_make
```

- [orb_slam3_ros](https://github.com/thien94/orb_slam3_ros_wrapper)
1. clone the repo and install dependencies

```
git clone https://github.com/thien94/orb_slam3_ros_wrapper.git
```
2. add launch file (rsd435_rgbd.launch) and yaml file, configuring to use realsense d435 camera
```
<node name="orb_slam3_rbgd" pkg="orb_slam3_ros_wrapper" type="orb_slam3_ros_wrapper_rgbd" output="screen">
    <remap from="/camera/rgb/image_raw"                 to="/uav1/rsd435/color/image_raw"/>
    <remap from="/camera/depth_registered/image_raw"    to="/uav1/rsd435/depth/image_raw"/>
    ...
</node>
```

## run
```
roslaunch ss_gazebo single_scanning.launch
roslaunch orb_slam3_ros_wrapper rsd435_rgbd.launch 
```



if gazebo is not killed, run
```
killall -9 gzserver
```