#!/usr/bin/env python
import rospy
import numpy as np
from math import *#sin, cos, acos, asin, radians

from geometry_msgs.msg import Pose

import transform

class viewpoint_creator:
    # initial vp_generator with camera information
    def __init__(self,camera):
        self.camera = camera

    # compute a viewpoint with input the offset on the image uv
    # and the transfom of point cloud coordinate frame to the world frame
    def compute_vp(self,dx,dy,dist,mat):
        depth = self.camera.depth_image()
        cx,cy = depth.center()
        u = cx+dx
        v = cy+dy
        vp = self._viewpoint(depth,u,v,dist,mat)
        return vp

    # create a viewpoint at pixel (u,v) and distance to surface is d
    # with tranform to the camere
    def _viewpoint(self,img,u,v,d,mat_c):
        # target point in camera frame at pixel u, v
        # print("current", mat)
        pc = self._position3d(img,u,v)
        if pc[2] < 0: # depth
            print("invalid depth image")
            return None

        #print("position at uv", pc)
        # target point normal in camera frame
        nc = self._normal3d_1(img,u,v)
        print("normal at uv", nc)
        pv = pc-d*nc
        #print("target vp", pv)
        # rotate normal in camera frame
        a = asin(-nc[1]) # about x
        b = atan2(nc[0],nc[2]) # about y
        c = 0 # about z
        mat_v = transform.rotation_translation(np.array([a,b,c]),pv)
        #print("vp matrix", mat_v)

        mat = np.dot(mat_c,mat_v)
        #print("vp in world", mat)
        vp = transform.matrix_to_cartesian(mat)
        return vp

    # evaluate pointion in 3d with the pixel u and v, in meter
    def _position3d(self,img,u,v):
        f = 0.001*self.camera.camera_focal() # in meter
        pp = self.camera.camera_principal()
        ps = 0.001*self.camera.pixel_size() # in meter
        dist = img.distance(u,v) # in meter
        #print("u,v",u,v,dist)
        x = dist*(u-pp[0])*ps/f[0]
        y = dist*(v-pp[1])*ps/f[1]
        z = dist
        return np.array([x,y,z])

    # calculate normal at pixel u,v
    def _normal3d(self,img,u,v):
        # z[v,u] for depth
        f = 0.001*self.camera.camera_focal()
        pp = self.camera.camera_principal()
        ps = 0.001*self.camera.pixel_size()
        cx = (u-pp[0])*ps
        cy = (v-pp[1])*ps
        z_uv = img.distance(u,v)
        dzdu = img.distance(u+10,v)-z_uv
        dzdv = img.distance(u,v+10)-z_uv
        dxdu = z_uv/f[0] + cx*dzdu/f[0]
        dydu = cy*dzdu/f[1]
        dxdv = cx*dzdv/f[0]
        dydv = z_uv/f[1] + cy*dzdv/f[1]
        vec_u = [dxdu,dydu,dzdu]
        vec_v = [dxdv,dydv,dzdv]
        n = np.cross(vec_u,vec_v) # v cross u for the normal pointing to the camera
        unit_n = n/np.linalg.norm(n)
        return unit_n

    def _normal3d_1(self,img,u,v):
        return img.normal(u,v,10)
