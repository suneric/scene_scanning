#!/usr/bin/env python3
import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor.image import DepthImage, ColorImage

# realsense d435
class RealSenseD435:
    # create a image view with a frame size for the ROI
    def __init__(self, robotNS=""):
        print("create realsense d435 instance...")
        self.robotNS=robotNS

        self.pixel = 0.003 # mm
        self.focal = np.array([1.93,1.93]) # mm
        self.principal = np.array([320,240]) # in pixel, could not be the center of the image
        self.imgSize = np.array([640,480])
        self.bridge=CvBridge()

        self.cameraInfoUpdate = False
        self.cvColor = None
        self.cvDepth = None
        self.points = None
        self.centerDist = -1.0
        self.create_camera_subs()

    def create_camera_subs(self):
        self.camInfoSub = rospy.Subscriber(self.robotNS+'/rsd435/color/camera_info', CameraInfo, self._caminfo_cb)
        self.depthSub = rospy.Subscriber(self.robotNS+'/rsd435/depth/image_raw', Image, self._depth_cb)
        self.colorSub = rospy.Subscriber(self.robotNS+'/rsd435/color/image_raw', Image, self._color_cb)
        self.pointSub = rospy.Subscriber(self.robotNS+'/rsd435/depth/points', PointCloud2, self._point_cb)

    def _point_cb(self,data):
        self.points = data

    def _caminfo_cb(self, data):
        if not self.cameraInfoUpdate:
            self.imgSize = np.array([data.width,data.height])
            self.focal = self.pixel*np.array([data.K[0],data.K[4]])
            self.principal = [data.K[2],data.K[5]]
            self.cameraInfoUpdate = True

    def _depth_cb(self, data):
        if not self.cameraInfoUpdate:
            print("camera is not ready.")
            return
        try:
            self.cvDepth = self.bridge.imgmsg_to_cv2(data, data.encoding) #"16UC1"
            img = DepthImage(self.cvDepth)
            c = self.image_center()
            self.centerDist = img.distance(c[0],c[0])
        except CvBridgeError as e:
            print(e)

    def _color_cb(self, data):
        if not self.cameraInfoUpdate:
            print("camera is not ready.")
            return
        try:
            self.cvColor = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img = ColorImage(self.cvColor)
            img.show(self.centerDist)
        except CvBridgeError as e:
            print(e)

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
        return self.imgSize/2
    def image_size(self):
        return self.imgSize
    def center_distance(self):
        return self.centerDist

    #### data
    def depth_image(self):
        return DepthImage(self.cvDepth)
    def color_image(self):
        return ColorImage(self.cvColor)
    def point_cloud(self):
        return self.points
