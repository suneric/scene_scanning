#!/usr/bin/env python3
import rospy
import numpy as np
from math import *
import pcl
from sensor.camera import RealSenseD435

#######################
class DataCapture:
    def __init__(self, camera, filepath):
        self.camera = camera
        self.ratio = self._frame_ratio()
        self.filepath = filepath
        self.index = 0

    ##################################################
    # scan and point could process
    # input point could, matrix of camera to global
    # pose: [quadrotor_pose, camera_joint]
    def scan_and_save(self,mat=None):
        pc = self.camera.point_cloud()
        if pc == None:
            return

        if mat:
            cloud = self._cloud_process(pc,mat)
        else:
            cloud = pc

        if cloud != None:
            print("save data.")
            self._save_cloud(cloud)
        else:
            print("no data captured.")

    ### private functions
    def _cloud_process(self,cloud,mat):
        # filter out the point outside of the frame and depth
        point_list = []
        for data in cloud:
            x,y,z,rgb=data[:4]
            tp = np.dot(mat,np.array([x,y,z,1]))
            point_list.append([tp[0,0],tp[0,1],tp[0,2],rgb])

        if len(point_list) > 0:
            pcl_cloud = pcl.PointCloud_PointXYZRGB()
            pcl_cloud.from_list(point_list)
            return pcl_cloud
        else:
            return None

    def _save_cloud(self,cloud):
        file = self.filepath+"point_"+str(self.index)+".pcd"
        pcl.save(cloud,file,"pcd")
        self.index = self.index + 1

    def _in_box(self,x,y,z, bbox):
        if x < bbox[0]:
            return False
        elif x > bbox[1]:
            return False
        elif y < bbox[2]:
            return False
        elif y > bbox[3]:
            return False
        elif z < bbox[4]:
            return False
        elif z > bbox[5]:
            return False
        else:
            return True

    def _frame_ratio(self):
        ps = self.camera.pixel_size()
        f = self.camera.camera_focal()
        frame = self.camera.image_size()
        x_ratio = frame[0]*ps/f[0]
        y_ratio = frame[1]*ps/f[1]
        return [x_ratio,y_ratio]

    def _bbox(self):
        # need to consider the case of zero center distance
        dist = self.camera.center_distance()
        x = dist*self.ratio[0]
        y = dist*self.ratio[1]
        return np.array([-x,x,-y,y,0.5*dist,1.5*dist])
