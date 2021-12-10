#!/usr/bin/env python
import rospy
import numpy as np
import os
import sys
from math import *#sin, cos, acos, asin, radians
from geometry_msgs.msg import Pose
import transform
from transform import QuadrotorTransform

class trajectory_defined:
    def __init__(self):
        self.trajectory = []
        self.index = 0
        self.transform_util = QuadrotorTransform()
        file = os.path.join(sys.path[0],'../../aircraft_scanning_plan/trajectory/uav/viewpoints.txt');
        #self._load_trajectory(file)
        self._create_trajectory()
        #self._create_wing_trajectory()


    def completed(self):
        return self.index >= len(self.trajectory)

    def next_view(self):
        if self.completed():
            return None
        else:
            cp = self.trajectory[self.index]
            self.index = self.index + 1
            return cp

    def _load_trajectory(self,file):
        print("load trajectory from", file)
        with open(file,'r') as reader:
            for line in reader.read().splitlines():
                data = line.split(" ")
                idx = int(data[0])
                px = float(data[1])
                py = float(data[2])
                pz = float(data[3])
                ox = float(data[4])
                oy = float(data[5])
                oz = float(data[6])
                ow = float(data[7])
                pose = Pose();
                pose.position.x = px
                pose.position.y = py
                pose.position.z = pz
                pose.orientation.x = ox
                pose.orientation.y = oy
                pose.orientation.z = oz
                pose.orientation.w = ow
                quadrotor, camera = self.transform_util.camera2quadrotor(pose)
                self.trajectory.append([quadrotor,camera])
        reader.close()


    def _create_wing_trajectory(self):
        self._create_intermediate(0,-28,9,0.5*pi,0)
        self._create_intermediate(-18,4,9,0.5*pi,0.5*pi)
        self._create_intermediate(-13,2,9,0.5*pi,0.5*pi)
        self._create_intermediate(-8,-2,9,0.5*pi,0.45*pi)
        self._create_intermediate(-8,2,7,-0.5*pi,0.45*pi)
        self._create_intermediate(-3,-3,11,0,0.4*pi)
        self._create_intermediate(-3,1,11,0,0.4*pi)
        self._create_intermediate(3,1,11,pi,0.4*pi)
        self._create_intermediate(3,-3,11,pi,0.4*pi)
        self._create_intermediate(8,-2,9,0.5*pi,0.45*pi)
        self._create_intermediate(8,2,7,-0.5*pi,0.45*pi)
        self._create_intermediate(13,2,9,0.5*pi,0.5*pi)
        self._create_intermediate(18,4,9,0.5*pi,0.5*pi)
        self._create_intermediate(0,-28,9,0.5*pi,0)

    def _create_trajectory(self):
        self._create_intermediate(0,-27,11,0.5*pi)
        # up part
        # head
        self._create_intermediate(-2,-29,2.5,0.3*pi,-0.1*pi)
        self._create_intermediate(0,-29,2.5,0.5*pi,-0.1*pi)
        self._create_intermediate(2,-29,2.5,0.7*pi,-0.1*pi)

        # self._create_intermediate(-3,25.5,13,0.0)
        # self._create_intermediate(0,29,13,-0.5*pi)
        # self._create_intermediate(3,25.5,13,pi)

        self._create_updownpath([-2,2],[-27,18],9.0,0.5*pi,1) # fuselage
        self._create_updownpath([-1,-1],[18,27],9.0,0.5*pi,1)
        self._create_updownpath([1,1],[27,18],9.0,0.5*pi,1)

        self._create_updownpath([2,8],[19,27],8.0,0.5*pi,1) # stablizer
        self._create_updownpath([-8,-2],[27,21],8.0,0.5*pi,1)

        self._create_updownpath([-20,-6],[6,0],8.0,0.5*pi,1) # wing
        self._create_updownpath([-6,-2],[0,-4],8.0,0.5*pi,1)
        self._create_updownpath([2,6],[-4,0],8.0,0.5*pi,1)
        self._create_updownpath([6,20],[0,6],8.0,0.5*pi,1)

        # down part
        self._create_updownpath([-2,2],[-27,-23],0.4,-0.5*pi,1) # fuselage
        self._create_intermediate(-2,-23,0.4,-0.5*pi)
        self._create_updownpath([-2,2],[-20,27],0.4,-0.5*pi,1) # fuselage

        self._create_updownpath([-20,-5],[0,5],0.4,-0.5*pi,1) # right wing
        self._create_updownpath([-8,8],[-7,0],0.4,-0.5*pi,1) # wing center
        self._create_intermediate(8,-1,0.4,-0.5*pi)
        self._create_updownpath([5,20],[0,5],0.4,-0.5*pi,1) # left wing
        self._create_updownpath([-8,8],[21,27],0.6,-0.5*pi,1) # stablizer
        self._create_intermediate(-10,26,0.5,0.5*pi)
        self._create_intermediate(-10,-26,0.5,0.5*pi)

        # tire
        self._create_intermediate(0,-26,0.5,0.5*pi)
        self._create_cylinder_path(0,-21.5,3.0,[1,2])
        self._create_intermediate(-10,-26,0.5,0.5*pi)
        self._create_intermediate(-10,1,0.5,0.5*pi)
        self._create_cylinder_path(-3.7,1,3.0,[0.5,1.2],0.6)
        self._create_intermediate(-10,4,0.5,0.5*pi)
        self._create_intermediate(0,4,0.5,0.5*pi)
        self._create_cylinder_path(3.7,1,3.0,[0.5,1.2],0.6)

        self._create_intermediate(2,-27,0.5,0.5*pi)
        self._create_intermediate(-2,-27,0.5,0.5*pi)

        # side fuselage
        self._create_sidepath_x([-2,2],-27,5,0.5*pi,0,2)
        self._create_sidepath_y(5.0,[-27,27],5,pi,0,3)
        self._create_sidepath_y(5.0,[27,-17],7,pi,0.2*pi,3)
        self._create_sidepath_x([2,-2],-27,7,0.5*pi,0.2*pi,2)
        self._create_sidepath_y(-5.0,[-27,27],7,0.0,0.2*pi,3)
        self._create_sidepath_x([-2,2],29,7,-0.5*pi,0,2)
        self._create_sidepath_x([2,-2],29,5,-0.5*pi,0,2)
        self._create_sidepath_y(-5.0,[27,-27],5,0.0,0,3)

        # side wing
        self._create_sidepath_x([-3,-9],-8,4.5,0.5*pi,0.0,1)
        self._create_sidepath_x([-9,-5],-10,2,0.5*pi,0.0,1)
        self._create_sidepath_x([-9,-20],-4,4.5,0.5*pi,0.0,2)
        self._create_sidepath_x([-20,-3],8,4,-0.5*pi,0.0,2)
        self._create_intermediate(-3,5,8,-0.5*pi)
        self._create_intermediate(3,5,8,-0.5*pi)
        self._create_sidepath_x([3,20],8,4,-0.5*pi,0.0,2)
        self._create_sidepath_x([20,9],-4,4.5,0.5*pi,0.0,2)
        self._create_sidepath_x([9,5],-10,2,0.5*pi,0.0,1)
        self._create_sidepath_x([9,3],-8,4.5,0.5*pi,0.0,1)

        # side stablizer
        self._create_sidepath_y(5,[20,26],6,pi,0.0,3)
        self._create_sidepath_y(5,[27,22],8,pi,0.0,3)
        self._create_sidepath_y(5,[24,28],10,pi,0.0,3)
        self._create_sidepath_x([0,0],30,12,-0.5*pi,0.0,2)
        self._create_sidepath_x([0,0],30,9,-0.5*pi,0.0,2)
        self._create_sidepath_x([0,0],30,5,-0.5*pi,0.0,2)
        self._create_sidepath_y(-5,[26,20],6,0,0.0,3)
        self._create_sidepath_y(-5,[22,27],8,0,0.0,3)
        self._create_sidepath_y(-5,[28,24],10,0,0.0,3)
        self._create_sidepath_x([0,0],14,7,0.5*pi,0.0,2)
        self._create_sidepath_x([0,0],16,9,0.5*pi,0.0,2)
        self._create_sidepath_x([0,0],18,11,0.5*pi,0.0,2)
        self._create_sidepath_x([0,0],20,13,0.5*pi,0.0,2)

        self._create_intermediate(-7,14,11,0.5*pi)
        self._create_intermediate(-7,14,2,0.5*pi)
        self._create_sidepath_x([-7,-7],3,2,-0.5*pi,0.0,1)
        self._create_sidepath_y(-11,[-1,-7],2,0.0,0.0,2)
        self._create_sidepath_x([-7,-7],-10,2,0.5*pi,0.0,1)
        self._create_sidepath_y(-3,[-7,-1],2,pi,0.0,2)

        self._create_sidepath_y(3,[-1,-7],2,0.0,0.0,2)
        self._create_sidepath_x([7,7],-10,2,0.5*pi,0.0,1)
        self._create_sidepath_y(11,[-7,-1],2,pi,0.0,2)
        self._create_sidepath_x([7,7],3,2,-0.5*pi,0.0,1)
        self._create_intermediate(7,14,2,-0.5*pi)
        self._create_intermediate(7,14,11,-0.5*pi)

        # back to the start
        self._create_intermediate(0,-27,11,-0.5*pi)


    def _create_intermediate(self,x,y,z,yaw,angle=0.0):
        vp = self._create_viewpoint(x,y,z,yaw,angle)
        self.trajectory.append(vp)

    def _create_updownpath(self,x_range,y_range,height,angle,offset):
        i = 0
        y_offset = 2*offset
        x_offset = offset
        ys,ye = y_range[0],y_range[1]+0.1
        if y_range[0] > y_range[1]:
            ys = y_range[0]
            ye = y_range[1]-0.1
            y_offset = -2

        for y in np.arange(ys,ye,y_offset):
            if np.mod(i,2) == 0:
                for x in np.arange(x_range[0],x_range[1]+0.1,x_offset):
                    vp = self._create_viewpoint(x,y,height,0.0,angle)
                    self.trajectory.append(vp)
            else:
                for x in np.arange(x_range[1],x_range[0]-0.1,-x_offset):
                    vp = self._create_viewpoint(x,y,height,0.0,angle)
                    self.trajectory.append(vp)
            i = i+1

    # path along y axis
    def _create_sidepath_y(self,x,y_range,z,yaw,angle,offset):
        y_offset = offset
        ys,ye = y_range[0],y_range[1]+0.1
        if y_range[0] > y_range[1]:
            ys = y_range[0]
            ye = y_range[1]-0.1
            y_offset = -2
        for y in np.arange(ys,ye,y_offset):
            vp = self._create_viewpoint(x,y,z,yaw,angle)
            self.trajectory.append(vp)

    # path along x axis
    def _create_sidepath_x(self,x_range,y,z,yaw,angle,offset):
        x_offset = offset
        xs,xe = x_range[0],x_range[1]+0.1
        if x_range[0] > x_range[1]:
            xs = x_range[0]
            xe = x_range[1]-0.1
            x_offset = -2
        for x in np.arange(xs,xe,x_offset):
            vp = self._create_viewpoint(x,y,z,yaw,angle)
            self.trajectory.append(vp)

        return

    def _create_cylinder_path(self,x,y,radius,z_range,z_offset=1.0):
        for theta in np.arange(0,2*pi,0.25*pi):
            for z in np.arange(z_range[0],z_range[1]+0.1,z_offset):
                vp = self._create_viewpoint(x-radius*cos(theta),y-radius*sin(theta),z,theta,0.0)
                self.trajectory.append(vp)

    def _create_viewpoint(self,x,y,z,yaw,theta):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qw,qx,qy,qz = transform.eularangle_to_quaternion(yaw, 0.0, 0.0)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return [pose, theta]
