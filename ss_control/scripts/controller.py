#!/usr/bin/env python
import rospy
import numpy as np
import time

import transform
from transform import QuadrotorTransform
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import JointControllerState
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Float64
from hector_uav_msgs.msg import PoseAction, TakeoffAction, LandingAction, PoseGoal, LandingGoal
from hector_uav_msgs.srv import EnableMotors

#######################
# controller for controlling the quadrotor pose and camera joint posetion
class UAVController:
    def __init__(self,robotName="uav", robotNS=''):
        self.robotName = robotName
        self.robotNS = robotNS

        self.camPose = None
        self.camPoseSub = self.create_camera_pose_sub()
        self.camPosePub = self.create_camera_pose_pub()

        self.uavPose = None
        self.uavPoseSub = self.create_uav_pose_sub()
        self.poseClient = self.create_pose_client()
        self.landingClient = self.create_landing_client()

        self.isTakeOff = False
        self.goal = None

        self.util = QuadrotorTransform()

    def create_camera_pose_sub(self):
        topic = self.robotNS+'/cam_control/position/state'
        return rospy.Subscriber(topic, JointControllerState, self._camera_pose_cb)

    def _camera_pose_cb(self, data):
        self.camPose = data.set_point

    def create_camera_pose_pub(self):
        topic = self.robotNS+'/cam_control/position/command'
        return rospy.Publisher(topic, Float64, queue_size=1)

    def create_uav_pose_sub(self):
        topic = '/gazebo/model_states'
        return rospy.Subscriber(topic, ModelStates, self._uav_cb)

    def _uav_cb(self, data):
        index = data.name.index(self.robotName)
        self.uavPose = data.pose[index]

    def create_pose_client(self):
        topic = self.robotNS+'/action/pose'
        return actionlib.SimpleActionClient(topic, PoseAction)

    def create_landing_client(self):
        topic = self.robotNS+'/action/landing'
        return actionlib.SimpleActionClient(topic, LandingAction)

    def _enable_motors(self, enable):
        service = self.robotNS+'/enable_motors'
        rospy.wait_for_service(service)
        srv = rospy.ServiceProxy(service, EnableMotors)
        srv.enable = enable
        print("enable motors", enable)

    def takeoff(self, callback):
        self.set_camera_joint(0.0)
        self._enable_motors('true')
        pose = self.uavPose
        pose.position.z = 1.0
        goal = PoseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.robotName+"/world"
        goal.target_pose.pose = pose
        self.poseClient.send_goal(goal)
        self.isTakeOff = True
        self.goal = pose
        rate = rospy.Rate(10)
        while not self.is_goal_reached():
            rate.sleep()
        callback()

    def landing(self, callback):
        self.set_camera_joint(0.0)
        goal = LandingGoal()
        self.landingClient.send_goal(goal)
        self.isTakeOff = False
        self.goal = None
        self._enable_motors('false')
        callback()

    def set_camera_joint(self,joint):
        if (joint < -1.5708):
            joint = -1.5708
        elif (joint > 1.5708):
            joint = 1.5708
        self.camPosePub.publish(joint)

    def fly_to(self, pose, callback):
        print("moving")
        goal = PoseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.robotName+"/world"
        goal.target_pose.pose = pose
        self.poseClient.send_goal(goal)
        self.goal = pose
        rate = rospy.Rate(10)
        while not self.is_goal_reached():
            rate.sleep()
        callback()

    def is_take_off(self):
        return self.isTakeOff

    def is_goal_reached(self):
        tolerance = 0.01
        if abs(self.uavPose.position.x - self.goal.position.x) > tolerance:
            return False
        elif abs(self.uavPose.position.y - self.goal.position.y) > tolerance:
            return False
        elif abs(self.uavPose.position.z - self.goal.position.z) > tolerance:
            return False
        else:
           return True

    def transform_q2c(self):
        mat = self.util.quadrotor2camera(self.uavPose,self.camPose)
        return mat

    def camera_pose(self):
        return self.camPose

    def uav_pose(self):
        return self.uavPose
