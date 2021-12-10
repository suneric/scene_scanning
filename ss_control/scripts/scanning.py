#!/usr/bin/env python
import rospy
import numpy as np
from math import *
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import transform

from controller import UAVController
from sensor.camera import RealSenseD435
from sensor.image import DepthImage, ColorImage


def create_trajectoy():
    trajectory = [(3.0,3.0,3.0,3.14,0.3),(3.0,0,3.0,3,0.3),(1.0,-3.0,3.0,1.57,0.3)]
    return trajectory


class AutoScanning:
    def __init__(self,controller,camera):
        self.controller = controller
        self.camera = camera

        self.isTakeoff = False
        self.isLanding = False
        self.status = 'ready'

        self.dataPub = self.create_data_pub(self.controller.robotName)

        self.trajectory = create_trajectoy()

    def create_data_pub(self, robotName):
        topic = robotName+'/pointcloud'
        return rospy.Publisher(topic, PointCloud2, queue_size=1)

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
            return
        goal = self.trajectory.pop(0)
        self._fly_to(goal)

    def _fly_to(self, goalPose):
        print("flying")
        self.status = 'flying'
        pose,angle = self.goal(goalPose)
        self.controller.set_camera_joint(angle)
        self.controller.fly_to(pose, self._fly_cb)

    def _fly_cb(self):
        print("reached.")
        self.status = 'scanning'
        self._scanning()
        self.status = 'ready'

    def _scanning(self):
        print("scanning.")
        pc = self.camera.point_cloud()
        if not pc:
            print("no data acuired.")
        else:
            self.dataPub.publish(pc)

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
    # initialize ros node
    robotName = 'uav1'
    robotNS = '/uav1'
    rospy.init_node("uav_scanning", anonymous=True, log_level=rospy.INFO)
    rospy.sleep(2)
    controller = UAVController(robotName=robotName, robotNS=robotNS)
    camera = RealSenseD435(robotNS=robotNS)
    auto = AutoScanning(controller, camera)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            if not auto.isTakeoff:
                rospy.sleep(1)
                auto.takeoff()
            else:
                key_input = raw_input("press enter for start autonomous flying:\n")
                if (key_input == 'l'):
                    if not auto.isLanding:
                        auto.landing()
                else:
                    auto.fly_and_scan()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
