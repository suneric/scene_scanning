#!/usr/bin/env python
import rospy
import numpy as np
from math import *
from geometry_msgs.msg import Pose

import transform
from trajectory import trajectory_defined
from controller import uav_cam_controller

from sensor.camera import realsense_d435
from sensor.image import depth_img, color_img
from sensor.pointcloud import data_capture

import os
import glob
import struct
import ctypes

class auto_scanning:
    def __init__(self,controller,camera,data_processor):
        self.controller = controller
        self.camera = camera
        self.data = data_processor

        self.takeoff = False
        self.landing = False
        self.status = 'ready' # 'completed', 'ready'

        self.trajectory = trajectory_defined()

    def is_takeoff(self):
        return self.takeoff
    def is_landing(self):
        return self.landing

    def start(self):
        self.controller.takeoff(self._takeoff_callback)

    def terminate(self):
        self.controller.landing()
        self.landing == True

    def fly(self):
        while True:
            if self.status == 'ready':
                self.status = 'flying'
                self._fly_to_next(self.trajectory.next_view())
            elif self.status == 'completed':
                self.terminate()

    def _fly_to_next(self, pose):
        if pose == None:
            self.status = 'completed'
            return
        print("flying to ", pose[0].position.x, pose[0].position.y, pose[0].position.z, "camera joint", pose[1])
        self.controller.execute_camera_joint(pose[1])
        self.controller.execute_quadrotor_pose(pose[0], self._fly_callback)

    def _fly_callback(self):
        print("reached.")
        mat = self.controller.transform_q2c()
        self.data.scan_and_save(mat)
        # self.trajectory.explore_views(current_pose)
        self.status = 'ready'

    def _takeoff_callback(self):
        print("takeoff.")
        pose = self._initial_pose(0,-30,8,0.5*pi,0)
        self.controller.execute_quadrotor_pose(pose, self._initial_callback)

    def _initial_callback(self):
        print("initialized.")
        self.takeoff = True

    def _initial_pose(self,x,y,z,yaw,angle):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qw,qx,qy,qz = transform.eularangle_to_quaternion(yaw, 0.0, 0.0)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.controller.execute_camera_joint(angle)
        return pose

# main
if __name__ == '__main__':
    # clean folder for save point cloud file
    temp_folder = "/home/yufeng/Temp/Scanning/"
    files = glob.glob(temp_folder+"*")
    for f in files:
        print("remove ", f)
        os.remove(f)

    # initialize ros node
    rospy.init_node("auto_scanning", anonymous=True, log_level=rospy.INFO)
    rospy.sleep(2)
    controller = uav_cam_controller(ns="agent1",robot_name="uav1")
    camera = realsense_d435(ns="agent1")
    pc_capture = data_capture(camera,temp_folder)
    auto = auto_scanning(controller, camera, pc_capture)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            if auto.is_takeoff() == False:
                rospy.sleep(1)
                auto.start()
            else:
                key_input = raw_input("press enter for start autonomous flying:\n")
                if (key_input == ''):
                    if auto.is_landing() == False:
                        auto.fly()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
