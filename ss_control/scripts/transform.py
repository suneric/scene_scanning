#!/usr/bin/env python3
import numpy as np
from numpy import pi, sqrt, cos, sin, arctan2, array, matrix
from numpy.linalg import norm
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from math import *

def trig(angle):
    return cos(angle),sin(angle)

# convert eular angle to quaternion
# yaw (Z)-pitch (Y)-roll (X)
# return q1,q1,q3,q4 which is w,x,y,z
def eularangle_to_quaternion(yaw, pitch, roll):
    # cy, sy = trig(0.5*yaw)
    # cp, sp = trig(0.5*pitch)
    # cr, sr = trig(0.5*roll)
    # w = sy*sp*sr+cy*cp*cr
    # x = -sy*sp*cr+cy*cp*sr
    # y = sy*cp*sr+cy*sp*cr
    # z = sy*cp*cr-cy*sp*sr
    q = quaternion_from_euler(roll,pitch,yaw)
    x,y,z,w = q[0],q[1],q[2],q[3]
    return w,x,y,z

def quaternion_to_eularangle(w,x,y,z):
    # sr_cp = 2*(w*x+y*z)
    # cr_cp = 1-2*(x*x+y*y)
    # roll = atan2(sr_cp, cr_cp)
    # sp = 2*(w*y-z*x)
    # pitch = asin(sp)
    # if abs(sp) >= 1:
    #     pitch = copysign(0.5*np.pi, sp)
    # sy_cp = 2*(w*z+x*y)
    # cy_cp = 1-2*(y*y+z*z)
    # yaw = atan2(sy_cp, cy_cp)
    euler = euler_from_quaternion([x,y,z,w])
    roll,pitch,yaw = euler[0],euler[1],euler[2]
    return yaw,pitch,roll

def Hrrt(ty, tz, l):
  cy, sy = trig(ty)
  cz, sz = trig(tz)
  return matrix([[cy * cz, -sz, sy * cz, 0.0],
                 [cy * sz, cz, sy * sz, 0.0],
                 [-sy, 0.0, cy, l],
                 [0.0, 0.0, 0.0, 1.0]])

def rr(p):
  ty = arctan2(sqrt(p[0,0]**2 + p[1,0]**2), p[2,0])
  tz = arctan2(p[1,0], p[0,0])

  if tz < -pi/2.0:
    ty = -ty
    tz += pi
  elif tz > pi/2.0:
    ty = -ty
    tz -= pi

  return (ty, tz)

def Rz(tz):
  (cz, sz) = trig(tz)
  return matrix([[ cz, -sz, 0.0],
                 [ sz,  cz, 0.0],
                 [0.0, 0.0, 1.0]])

def Ryz(ty, tz):
  (cy, sy) = trig(ty)
  (cz, sz) = trig(tz)
  return matrix([[cy * cz, -sz, sy * cz],
                 [cy * sz, cz, sy * sz],
                 [-sy, 0.0, cy]])

def rotation_translation(rotation, translation):
    Cx,Sx = trig(rotation[0])
    Cy,Sy = trig(rotation[1])
    Cz,Sz = trig(rotation[2])
    dX = translation[0]
    dY = translation[1]
    dZ = translation[2]
    mat_trans = matrix([[1,0,0,dX],
                          [0,1,0,dY],
                          [0,0,1,dZ],
                          [0,0,0,1]])
    mat_rotX = matrix([[1,0,0,0],
                         [0,Cx,-Sx,0],
                         [0,Sx,Cx,0],
                         [0,0,0,1]])
    mat_rotY = matrix([[Cy,0,Sy,0],
                         [0,1,0,0],
                         [-Sy,0,Cy,0],
                         [0,0,0,1]])
    mat_rotZ = matrix([[Cz,-Sz,0,0],
                         [Sz,Cz,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
    return mat_rotZ*mat_rotY*mat_rotX*mat_trans

def cartesian_to_matrix(cp):
    position = cp.position
    orientation = cp.orientation
    mat = np.eye(4)
    # translation
    mat[0,3] = position.x# in meter
    mat[1,3] = position.y
    mat[2,3] = position.z
    # quaternion to matrix
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    R = quaternion_matrix([x,y,z,w])
    mat[0:3,0:3] = R[0:3,0:3]
    mat[3,3] = 1
    return mat

# homougenous matrix to quaternion
# return Pose()
def matrix_to_cartesian(mat):
    cp = Pose()
    cp.position.x = mat[0,3]
    cp.position.y = mat[1,3]
    cp.position.z = mat[2,3]
    q = quaternion_from_matrix(mat)
    cp.orientation.x = q[0]
    cp.orientation.y = q[1]
    cp.orientation.z = q[2]
    cp.orientation.w = q[3]
    return cp

# return Pose()
def eular_to_cartesian(self,p,a,b,c):
    cp = Pose()
    cp.position.x = p[0]
    cp.position.y = p[1]
    cp.position.z = p[2]
    w,x,y,z = eularangle_to_quaternion(c,b,a)
    cp.orientation.w = w
    cp.orientation.x = x
    cp.orientation.y = y
    cp.orientation.z = z
    return cp


##############################################################################
# Transform utility of Mobile Manipulator
class MMRobotTransform:
    def __init__(self):
        self.arm_base = [-0.35,0,0.7] # arm base offset in mobile base frame
        self.arm_l02 = 0.36 # arm length from joint 0-2
        self.arm_l24 = 0.42 # arm length from joint 2-4
        self.arm_l46 = 0.4 # arm length from joint 4-6
        self.arm_l6E = 0.126 # arm length from joint 6-endeffector
        self.camera_depth = 0.0208 # arm length from endeffector to camere
        self.tr = 0.0 # redundancy
        self.mobile_base_height = 0.15

    # forward matrix from mobile base to camera
    def mobilebase2camera(self, ugv_pos, arm_pos):
        mat0 = cartesian_to_matrix(ugv_pos)
        mat1 = self.mobilebase2armbase()
        mat2 = self.armbase2camera(arm_pos)
        mat3 = self.camera2pointcloud()
        return mat0*mat1*mat2*mat3

    def mobilebase2armbase(self):
        return matrix([[1,0,0,self.arm_base[0]],
                         [0,1,0,self.arm_base[1]],
                         [0,0,1,self.arm_base[2]],
                         [0,0,0,1]])

    def armbase2camera(self,joints):
        H02 = Hrrt(joints[1],joints[0],self.arm_l02)
        H24 = Hrrt(-joints[3],joints[2],self.arm_l24)
        H46 = Hrrt(joints[5],joints[4],self.arm_l46)
        H6E = Hrrt(0.0,joints[6],self.arm_l6E+self.camera_depth)
        H0E = H02 * H24 * H46 * H6E
        return H0E

    def camera2pointcloud(self):
        # the pointcloud coordinate system is -0.5*pi rotated about z axis
        mat_rz = matrix([[cos(-0.5*pi),-sin(-0.5*pi),0,0],
                          [sin(-0.5*pi),cos(-0.5*pi),0,0],
                          [0,0,1,0],
                          [0,0,0,1]])
        return mat_rz

    # reverse transform from camera to mobile base
    # camera is a cartesian position in world coordinate system
    def camera2mobilebase(self,camera):
        cam_euler = euler_from_quaternion([camera.orientation.x,camera.orientation.y,camera.orientation.z,camera.orientation.w])
        roll,pitch,yaw = cam_euler[0],cam_euler[1],cam_euler[2]
        #print("yaw,pitch,roll",yaw,pitch,roll)
        # use joint 6 angle to control the camera pitch
        # since the final camera coordinate is rotated -0.5*pi about z axis
        # the roll is the joint 6 angle, while it is range or [-0.5*pi,0.5*pi]
        j6 = -roll
        joints = [0,0,0,0,0,j6,0]
        matvp = self.relocate_camera(j6,camera)
        mat1 = self.mobilebase2armbase()
        mat2 = self.armbase2camera(joints)
        mat3 = self.camera2pointcloud()
        mat0 = matvp*np.linalg.inv(mat1*mat2*mat3)
        ugv = matrix_to_cartesian(mat0)
        px,py,pz = ugv.position.x,ugv.position.y,ugv.position.z
        euler = euler_from_quaternion([ugv.orientation.x,ugv.orientation.y,ugv.orientation.z,ugv.orientation.z])
        ugv_pose = (px,py,euler[2])
        return ugv_pose, joints

    def relocate_camera(self,j6,vp):
        # determine the z of camera, as the limited workspace of mobile manipulator
        arm_base_z = self.mobile_base_height+self.arm_base[2]
        arm_l06 = self.arm_l02+self.arm_l24+self.arm_l46
        arm_6C = self.arm_l6E+self.camera_depth
        cam_z = arm_base_z+arm_l06+arm_6C*cos(j6)

        matvp = cartesian_to_matrix(vp)
        xyz = np.array([matvp[0,3],matvp[1,3],matvp[2,3]])
        nm = np.array([matvp[2,0],matvp[2,1],matvp[2,2]])
        dist = abs(xyz[2]-cam_z)/cos(j6)
        nvp = xyz
        if xyz[2] > cam_z:
            nvp = xyz-nm*dist
        elif xyz[2] < cam_z:
            nvp = xyz+nm*dist
        # print(nvp)
        # camera pose matrix
        matnvp = np.eye(4)
        matnvp[0,3],matnvp[1,3],matnvp[2,3] = nvp[0],nvp[1],nvp[2]
        matnvp[0:3,0:3] = matvp[0:3,0:3]
        matnvp[3,3] = 1
        return matnvp

    # give a camera position in arm coordinate system
    def cartesian2joint(self,vp):
        t = 7*[0.0]
        pE0 = matrix([[vp.position.x],
                      [vp.position.y],
                      [vp.position.z]])
        qE0 = array([vp.orientation.x,
                     vp.orientation.y,
                     vp.orientation.z,
                     vp.orientation.w])
        pE6 = matrix([[0.0], [0.0], [self.arm_l6E+self.camera_depth]])
        p20 = matrix([[0.0], [0.0], [self.arm_l02]])
        RE0 = matrix(quaternion_matrix(qE0)[:3,:3])
        p6E0 = RE0*pE6
        p60 = pE0-p6E0
        p260 = p60-p20

        s = norm(p260)
        if s > self.arm_l24 + self.arm_l46:
          print('invalid pose command')
          return [0,0,0,0,0,0,0]

        (tys, tzs) = rr(p260)
        tp24z0 = 1/(2.0 * s) * (self.arm_l24**2 - self.arm_l46**2 + s**2)
        tp240 = matrix([[-sqrt(self.arm_l24**2 - tp24z0**2)], [0.0], [tp24z0]])
        p240 = Ryz(tys, tzs) * Rz(self.tr) * tp240
        (t[1], t[0]) = rr(p240)

        R20 = Ryz(t[1], t[0])
        p40 = p20 + p240
        p460 = p60 - p40
        p462 = R20.T * p460
        (t[3], t[2]) = rr(p462)
        t[3] = -t[3]

        R42 = Ryz(-t[3], t[2])
        R40 = R20 * R42
        p6E4 = R40.T * p6E0
        (t[5], t[4]) = rr(p6E4)

        R64 = Ryz(t[5], t[4])
        R60 = R40 * R64
        RE6 = R60.T * RE0
        t[6] = arctan2(RE6[1,0], RE6[0,0])
        return t

##############################################################################
# transform utility of quadrotor
class QuadrotorTransform:
    def __init__(self):
        self.cam_base = [0.42,0.0,0.0] # offset of camera base
        self.cam_len = 0.0358 # length from joint to camera

    # forward transform from quadrotor to camera
    def quadrotor2camera(self, pose, angle):
        # quadrotor position matrix
        mat0 = cartesian_to_matrix(pose)
        mat1 = self.camerabase()
        mat2 = self.camerabase2camera(angle)
        mat3 = self.camera2pointcloud()
        return mat0*mat1*mat2*mat3

    def camerabase(self):
        # camera joint translation in base frame
        return matrix([[1,0,0,self.cam_base[0]],
                         [0,1,0,self.cam_base[1]],
                         [0,0,1,self.cam_base[2]],
                         [0,0,0,1]])

    def camerabase2camera(self,angle):
        # rs435 camera rotation about y axis, translate along x axis
        return matrix([[cos(angle),0,sin(angle),self.cam_len],
                         [0,1,0,0],
                         [-sin(angle),0,cos(angle),0],
                         [0,0,0,1]])

    def camera2pointcloud(self):
        # point cloud frame respect to rs435 frame
        mat_z = matrix([[cos(-0.5*pi),-sin(-0.5*pi),0,0],
                          [sin(-0.5*pi),cos(-0.5*pi),0,0],
                          [0,0,1,0],
                          [0,0,0,1]])
        mat_x = matrix([[1,0,0,0],
                          [0,cos(-0.5*pi),-sin(-0.5*pi),0],
                          [0,sin(-0.5*pi),cos(-0.5*pi), 0],
                          [0,0,0,1]])
        return mat_z*mat_x

    # decompose the viewpoint to quadrotor position and angle of camera joint
    def camera2quadrotor(self,camera):
        # as the camera rs_d435 rotates about z axis -pi/2
        eular_angle = euler_from_quaternion([camera.orientation.x,camera.orientation.y,camera.orientation.z,camera.orientation.w])
        roll = eular_angle[0]
        pitch = eular_angle[1]
        yaw = eular_angle[2]
        #print("yaw,pitch,roll",yaw,pitch,roll)
        # the camera coordinate is x-right, y-down, and z-forward
        # the roll should negative [-pi,0],the camere joint angle is in range [-0.5*PI, 0.5*PI]
        # the angle for camera joint should be -roll - 0.5*pi
        angle = -roll-0.5*pi
        matvp = cartesian_to_matrix(camera)
        mat1 = self.camerabase()
        mat2 = self.camerabase2camera(angle)
        mat3 = self.camera2pointcloud()
        mat0 = matvp*np.linalg.inv(mat1*mat2*mat3)
        quadpose = matrix_to_cartesian(mat0)
        return quadpose, angle
