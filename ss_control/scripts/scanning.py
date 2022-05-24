#!/usr/bin/env python3
import rospy
import numpy as np
from math import *
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import sensor_msgs.point_cloud2 as pc2
import transform
from transform import QuadrotorTransform
from controller import UAVController
from sensor.camera import RealSenseD435
from sensor.image import DepthImage, ColorImage
from sensor.pointcloud import DataCapture
import os
import sys
import glob
from trajectory import scanning_path

class AutoScanning:
    def __init__(self,controller,camera,scanpath,datacap):
        self.controller = controller
        self.camera = camera

        self.isTakeoff = False
        self.isLanding = False
        self.status = 'ready'

        self.dataPub = self.create_data_pub(self.controller.robotName)
        self.posePub = self.create_pose_pub(self.controller.robotName)

        self.trajectory = scanpath.trajectory
        self.viewpoint_count = len(self.trajectory)
        self.datacap = datacap

    def create_data_pub(self, robotName):
        topic = robotName+'/pointcloud'
        return rospy.Publisher(topic, PointCloud2, queue_size=1)

    def create_pose_pub(self, robotName):
        topic = robotName+'/viewpoint'
        return rospy.Publisher(topic, Float64MultiArray, queue_size=1)

    def landing(self):
        self.controller.landing(self._landing_cb)

    def _landing_cb(self):
        print("landing...")
        self.isLanding == True
        self.status = "done"

    def takeoff(self):
        self.controller.takeoff(self._takeoff_cb)

    def _takeoff_cb(self):
        print("takeoff...")
        self.isTakeoff = True
        self.status = 'ready'

    def fly_and_scan(self):
        if self.status != 'ready':
            return
        if len(self.trajectory) == 0:
            self.status = 'compelete'
            self.landing()
            return
        print("{} th viewpoint".format(self.viewpoint_count-len(self.trajectory)+1))
        goal = self.trajectory.pop(0)
        self._fly_to(goal)

    def _fly_to(self, goalPose):
        print("flying")
        self.status = 'flying'
        pose,angle = goalPose[0],goalPose[1] #self.goal(goalPose)
        self.controller.set_camera_joint(angle)
        self.controller.fly_to(pose, self._fly_cb)

    def _fly_cb(self):
        print("reached.")
        self.status = 'scanning'
        self._scanning()
        self.status = 'ready'

    def _scanning(self):
        print("scanning.")
        self.datacap.scan_and_save(self.controller.transform_q2c())

        vp = self.viewpoint()
        pc = self.pointcloud()
        if not pc:
            print("no data acuired.")
        else:
            self.posePub.publish(vp)
            self.dataPub.publish(pc)

    def viewpoint(self):
        mat = self.controller.transform_q2c()
        vp = Float64MultiArray()
        vp.layout.dim.append(MultiArrayDimension())
        vp.layout.dim.append(MultiArrayDimension())
        vp.layout.dim[0].size=4
        vp.layout.dim[1].size=4
        vp.layout.dim[0].stride=4*4
        vp.layout.dim[1].stride=4
        vp.layout.data_offset=0
        vp.data = mat.flatten().tolist()[0]
        return vp

    def pointcloud(self):
        return self.camera.point_cloud()

    def goal(self,goalPose):
        pose = Pose()
        pose.position.x = goalPose[0]
        pose.position.y = goalPose[1]
        pose.position.z = goalPose[2]
        qw,qx,qy,qz = transform.eularangle_to_quaternion(goalPose[3], 0.0, 0.0)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose, goalPose[4]

if __name__ == '__main__':
     # clean folder for save point cloud file
    temp_folder = os.path.join(sys.path[0],'..','data/temp/')
    files = glob.glob(temp_folder+"*")
    for f in files:
        print("remove ", f)
        os.remove(f)

    # initialize ros node
    robotName = 'uav1'
    robotNS = '/uav1'
    rospy.init_node("uav_scanning", anonymous=True, log_level=rospy.INFO)
    rospy.sleep(2)
    controller = UAVController(robotName=robotName, robotNS=robotNS)
    camera = RealSenseD435(robotNS=robotNS)
    scanpath = scanning_path()
    datacap = DataCapture(camera, temp_folder)
    auto = AutoScanning(controller, camera, scanpath, datacap)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            if not auto.isTakeoff:
                rospy.sleep(1)
                auto.takeoff()
            else:
                if not auto.isLanding:
                    auto.fly_and_scan()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
