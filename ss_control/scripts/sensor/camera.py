#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from image import depth_img, color_img

# realsense d435
class realsense_d435:
    # create a image view with a frame size for the ROI
    def __init__(self, ns="agent1"):
        print("create realsense d435 instance...")
        # camera information
        self.ns=ns
        self.cameraInfoUpdate = False
        self.pixel = 0.003 # mm
        self.focal = np.array([1.93,1.93]) # mm
        self.principal = np.array([320,240]) # in pixel, could not be the center of the image
        self.img_size = np.array([640,480])

        self.bridge=CvBridge()
        # ros-realsense
        self.caminfo_sub = rospy.Subscriber(self.ns+'/rs435/color/camera_info', CameraInfo, self._caminfo_callback)
        self.depth_sub = rospy.Subscriber(self.ns+'/rs435/depth/image_raw', Image, self._depth_callback)
        self.color_sub = rospy.Subscriber(self.ns+'/rs435/color/image_raw', Image, self._color_callback)
        self.point_sub = rospy.Subscriber(self.ns+'/rs435/depth/points', PointCloud2, self._point_callback)

        # data
        self.cv_color = None
        self.cv_depth = None
        self.points = None
        self.center_dist = -1.0

    def ready(self):
        return self.cameraInfoUpdate

    #### camera properties
    def camera_focal(self): # in mm
        return self.focal
    def camera_principal(self): # origin
        return self.principal
    def pixel_size(self): # in mm
        return self.pixel

    #### image properties
    # center of the image frame
    def image_center(self):
        return self.img_size/2
    def image_size(self):
        return self.img_size
    def center_distance(self):
        return self.center_dist

    #### data
    def depth_image(self):
        return depth_img(self.cv_depth)
    def color_image(self):
        return color_img(self.cv_color)
    def point_cloud(self):
        return pc2.read_points(self.points,skip_nans=True)

    # private functions
    def _point_callback(self,data):
        self.points = data

    def _caminfo_callback(self, data):
        if self.cameraInfoUpdate == False:
            self.img_size = np.array([data.width,data.height])
            self.focal = self.pixel*np.array([data.K[0],data.K[4]])
            self.principal = [data.K[2],data.K[5]]
            self.cameraInfoUpdate = True

    def _depth_callback(self, data):
        if self.cameraInfoUpdate:
            try:
                self.cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding) #"16UC1"
                img = depth_img(self.cv_depth)
                c = self.image_center()
                self.center_dist = img.distance(c[0],c[0])
            except CvBridgeError as e:
                print(e)

    def _color_callback(self, data):
        if self.cameraInfoUpdate:
            try:
                self.cv_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
                img = color_img(self.cv_color)
                img.show(self.center_dist)
            except CvBridgeError as e:
                print(e)
