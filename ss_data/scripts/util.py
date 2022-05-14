#! /usr/bin/env python
import sys
import os
import numpy as np
from math import *
from geometry_msgs.msg import Pose
import copy

from tf.transformations import *

MAXVALUE = 1000000

"""
ViewPoint for camere position with voxels in the view
"""
class ViewPoint(object):
    def __init__(self,id,px,py,pz,ox,oy,oz,ow):
        self.id = id
        self.voxels = set() # grid / voxels in view
        self.camera = self.cameraPose(px,py,pz,ox,oy,oz,ow)

    def cameraPose(self,px,py,pz,ox,oy,oz,ow):
        camera = Pose()
        camera.position.x = px
        camera.position.y = py
        camera.position.z = pz
        camera.orientation.x = ox
        camera.orientation.y = oy
        camera.orientation.z = oz
        camera.orientation.w = ow
        return camera

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

"""
utility function of viewpoint
"""
def vpDistance(vp1, vp2):
    x1 = vp1.camera.position.x;
    y1 = vp1.camera.position.y;
    z1 = vp1.camera.position.z;
    x2 = vp2.camera.position.x;
    y2 = vp2.camera.position.y;
    z2 = vp2.camera.position.z;
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))

def vpOverlap(vp1, vp2):
    intersect = vp1.voxels & vp2.voxels
    return len(intersect)

def vpDirection(vp1, vp2):
    x = vp2.camera.position.x - vp1.camera.position.x
    y = vp2.camera.position.y - vp1.camera.position.y
    z = vp2.camera.position.z - vp1.camera.position.z
    return [x,y,z]

def angle(v1, v2):
    d1 = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2])
    d2 = sqrt(v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2])
    cos_a = float(v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])/float(d1*d2)
    sign = np.sign(float(v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]))
    return sign*np.arccos(cos_a)*180/3.14

def nearestViewpoint(pos, vps):
    x, y, z = pos[0],pos[1], pos[2]
    minDist = MAXVALUE
    idx = 0
    for i in range(len(vps)):
        vpx = vps[i].camera.position.x
        vpy = vps[i].camera.position.y
        vpz = vps[i].camera.position.z
        dist = sqrt((x-vpx)*(x-vpx)+(y-vpy)*(y-vpy)+(z-vpz)*(z-vpz))
        if dist < minDist:
            minDist = dist
            idx = i
    print("nearest viewpoint index is {} for pos ({},{},{})".format(i,x,y,z))
    return i

"""
A problem with Nearest Neighbors is outliers. It seems that picking up an outlier is sometimes
a good idea, but sometimes going directly to the nearest neighbor is a better idea. It is
difficult to make the choice between an outlier and a nearest neighbor while awe are constructing
a tour, because we don't have the context of the whole tour yet. One way we could try to imporve
a tour us by reversing a segement
"""
def reverseSegementIfBetter(tour,i,j):
    """If reversing tour[i:j] would make the tour shorter, then do it."""
    A,B,C,D = tour[i-1],tour[i],tour[j-1],tour[j%len(tour)]
    if vpDistance(A,B)+vpDistance(C,D) > vpDistance(A,C)+vpDistance(B,D):
        tour[i:j]=reversed(tour[i:j])

def allSegments(N):
    return [(start,start+length) for length in range(N,2-1,-1) for start in range(N-length+1)]

def tourLength(tour):
    return sum(vpDistance(tour[i],tour[i-1]) for i in range(len(tour)))

def alterTour(tour):
    original_length = tourLength(tour)
    for (start,end) in allSegments(len(tour)):
        reverseSegementIfBetter(tour,start,end)
    if tourLength(tour) < original_length:
        return alterTour(tour)
    return tour

def mirrorTour(tour):
    """reflect plane y-z point (0,0,0), normal (1,0,0)"""
    M0 = reflection_matrix((0,0,0), (1,0,0))
    mvps = []
    for vp in tour:
        p = vp.camera.position
        q = vp.camera.orientation
        print(p,q)
        mp = np.array((p.x,p.y,p.z,1.0))*np.matrix(M0)
        mp = np.squeeze(np.asarray(mp))
        # mq = quaternion_matrix((q.x,q.y,q.z,q.w))*np.matrix(M0)
        # mq = quaternion_from_matrix(mq)
        mq = (q.x,-q.y,-q.z,q.w)
        # print(mp,mq)
        mvp = ViewPoint(vp.id+10000,mp[0],mp[1],mp[2],mq[0],mq[1],mq[2],mq[3])
        mvps.append(mvp)
    return mvps


class ViewPointUtil2(object):
    def __init__(self, vps, overlap=0.1):
        self.voxels = set()
        self.viewpoints = []
        self.initialViewpoints(vps)
        self.nbMap = []
        self.overlap = overlap

    def initialViewpoints(self,vps):
        self.originalvps = vps
        allVoxels = set()
        for vp in self.originalvps:
            allVoxels |= vp.voxels
        # update voxels for saving memory
        allVoxelsList = list(allVoxels)
        for vp in self.originalvps:
            newVoxels = set()
            for v in vp.voxels:
                vIdx = allVoxelsList.index(v)
                self.voxels.add(vIdx)
                newVoxels.add(vIdx)
            newVp = copy.deepcopy(vp)
            newVp.voxels = newVoxels
            self.viewpoints.append(newVp)
        return

    def originalViewpoint(self,vp):
        vpIdx = self.viewpoints.index(vp)
        return self.originalvps[vpIdx]

    def neighbors(self,vpIdx):
        return self.nbMap[vpIdx]

    """
    build a map for viewpoints
    """
    def buildNeighborMap(self):
        print("=== start building neighbor map ===".format())
        dim = len(self.viewpoints)
        self.nbMap = [None]*dim
        for i in range(dim):
            nbvps = self.searchNeighbor(i,self.viewpoints,self.overlap)
            self.nbMap[i] = nbvps
        print("=== end building neighbor map ===".format())
        return self.nbMap

    """
    search neighbor viewpoints of a given viewpoint
    considering the distance and overlap
    """
    def searchNeighbor(self,i,vps,overlap):
        dim = len(vps)
        vp = vps[i]

        # matrix of camera
        mat = cartesian_to_matrix(vp.camera)
        refx = [mat[0][0],mat[1][0],mat[2][0]] # x-axis

        right_vps,left_vps,up_vps,down_vps = [],[],[],[]
        dists = []
        for j in range(dim):
            vp1 = vps[j]
            # skip same vp
            if vp1.id == vp.id:
                continue
            # check overlap ratio
            duplicate = float(vpOverlap(vp, vp1) / (len(vp.voxels)+len(vp1.voxels)))
            if duplicate < overlap:
                continue

            dir = vpDirection(vp,vp1)
            a = angle(dir,refx)
            if a > -45 and a <=45:
                right_vps.append(j)
            elif a > 45 and a <= 135:
                up_vps.append(j)
            elif a > -135 or a <= -45:
                down_vps.append(j)
            else:
                left_vps.append(j)

        right_vp = self.findFarestNeighbor(i,vps,right_vps)
        up_vp = self.findFarestNeighbor(i,vps,up_vps)
        left_vp = self.findFarestNeighbor(i,vps,left_vps)
        down_vp = self.findFarestNeighbor(i,vps,down_vps)
        nbvps = [right_vp,up_vp,left_vp,down_vp]
        print("neighborhood vps",nbvps)
        return nbvps

    def findFarestNeighbor(self,i,vps,indices):
        vpIdx = i
        dist = 0.0
        for idx in indices:
            vpDist = vpDistance(vps[i],vps[idx])
            if vpDist > dist:
                dist = vpDist
                vpIdx = idx
        return vpIdx

class ViewPointUtil(object):
    def __init__(self, vps, actDim=None, cn=0.3):
        self.voxels = set()
        self.viewpoints = []
        self.initialViewpoints(vps)
        self.nbMap = []
        self.actDim = actDim
        self.cn = cn

    def neighbors(self,vpIdx,size=0):
        if size == 0:
            return self.nbMap[vpIdx]
        else:
            return self.nbMap[vpIdx][0:size]

    def initialViewpoints(self,vps):
        self.originalvps = vps
        allVoxels = set()
        for vp in self.originalvps:
            allVoxels |= vp.voxels

        # update voxels for saving memory
        allVoxelsList = list(allVoxels)
        for vp in self.originalvps:
            newVoxels = set()
            for v in vp.voxels:
                vIdx = allVoxelsList.index(v)
                self.voxels.add(vIdx)
                newVoxels.add(vIdx)
            newVp = copy.deepcopy(vp)
            newVp.voxels = newVoxels
            self.viewpoints.append(newVp)
        return

    def originalViewpoint(self,vp):
        vpIdx = self.viewpoints.index(vp)
        return self.originalvps[vpIdx]

    """
    build a map for viewpoints
    """
    def buildNeighborMap(self):
        print("=== start building neighbor map ===".format())
        dim = len(self.viewpoints)
        self.nbMap = [None]*dim
        for i in range(dim):
            nbvps = self.searchNeighbor(i,self.viewpoints,self.cn)
            self.nbMap[i] = nbvps
        print("=== end building neighbor map ===".format())
        return self.nbMap

    """
    search neighbor viewpoints of a given viewpoint
    considering the distance and overlap
    """
    def searchNeighbor(self,i,vps,cn):
        dim = len(vps)
        vp = vps[i]
        scoreList = [None]*dim
        for j in range(dim):
            vp1 = vps[j]
            if vp1.id == vp.id:
                scoreList[j] = MAXVALUE
            else:
                dist = vpDistance(vp, vp1)
                overlap = vpOverlap(vp, vp1)
                scoreList[j]=cn*dist + (1.0-cn)*overlap
        # return the viewpoint indices with least value
        sortedIndice = np.argsort(np.array(scoreList))
        # sorted = np.lexsort((range(dim), scoreList))[0:self.actDim]
        return sortedIndice
