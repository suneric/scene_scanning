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
class uav_cam_controller:
    def __init__(self,robot_name="uav1",ns="agent1"):
        self.robot_name = robot_name
        self.ns = ns
        self.camerapose = None
        self.transform_util = QuadrotorTransform()
        self.camerapose_sub = rospy.Subscriber(ns+'/camera_controller/state', JointControllerState, self._camerapose_callback)
        self.camerapose_pub = rospy.Publisher(ns+'/camera_controller/command', Float64, queue_size=1)

        self.pose_client = actionlib.SimpleActionClient(ns+"/action/pose", PoseAction)
        self.landing_client = actionlib.SimpleActionClient(ns+"/action/landing", LandingAction)

        self.quadrotor_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._quadrotor_callback)
        self.quadrotor_pose = None
        self.b_take_off = False
        self.goal_pose = None

    def transform_q2c(self):
        mat = self.transform_util.quadrotor2camera(self.quadrotor_pose,self.camerapose)
        return mat

    def camera_pose(self):
        return self.camerapose
    def uav_pose(self):
        return self.quadrotor_pose

    def is_take_off(self):
        return self.b_take_off

    def takeoff(self, callback):
        self.execute_camera_joint(1.0)

        self._enable_motors('true')
        pose = self.quadrotor_pose
        pose.position.z = 1.0
        goal = PoseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.pose = pose
        self.pose_client.send_goal(goal)

        self.goal_pose = pose
        self.b_take_off = True

        rate = rospy.Rate(10)
        while not self.is_goal_reached():
            rate.sleep()
        callback()

    def landing(self):
        self.execute_camera_joint(0.0)

        goal = LandingGoal()
        self.landing_client.send_goal(goal)

        self.b_take_off = False
        self.goal_pose = None

        self._enable_motors('false')

    def execute_camera_joint(self,joint):
        if (joint < -1.5708):
            joint = -1.5708
        elif (joint > 1.5708):
            joint = 1.5708
        self.camerapose_pub.publish(joint)

    def execute_quadrotor_pose(self, pose, callback):
        print("moving")
        goal = PoseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.pose = pose
        self.pose_client.send_goal(goal)
        self.goal_pose = pose
        rate = rospy.Rate(10)
        while not self.is_goal_reached():
            rate.sleep()
        callback()

    ### private functions
    def _enable_motors(self, enable):
        rospy.wait_for_service(self.ns+'/enable_motors')
        srv = rospy.ServiceProxy(self.ns+'/enable_motors', EnableMotors)
        srv.enable = enable
        print("enable motors")

    def _quadrotor_callback(self, data):
        index = data.name.index(self.robot_name) # the quadrotor pose index in /gazebo/model_states
        self.quadrotor_pose = data.pose[index]

    def _camerapose_callback(self, data):
        self.camerapose = data.set_point

    def is_goal_reached(self):
        tolerance = 0.01
        if abs(self.quadrotor_pose.position.x - self.goal_pose.position.x) > tolerance:
            return False
        elif abs(self.quadrotor_pose.position.y - self.goal_pose.position.y) > tolerance:
            return False
        elif abs(self.quadrotor_pose.position.z - self.goal_pose.position.z) > tolerance:
            return False
        # elif abs(self.quadrotor_pose.orientation.x - self.goal_pose.orientation.x) > tolerance:
        #     return False
        # elif abs(self.quadrotor_pose.orientation.y - self.goal_pose.orientation.y) > tolerance:
        #     return False
        # elif abs(self.quadrotor_pose.orientation.z - self.goal_pose.orientation.z) > tolerance:
        #     return False
        # elif abs(self.quadrotor_pose.orientation.w - self.goal_pose.orientation.w) > tolerance:
        #    return False
        else:
           return True
