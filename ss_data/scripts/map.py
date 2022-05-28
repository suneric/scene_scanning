#
import random
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import math
from util import *
import sys
import os
from geometry_msgs.msg import Pose

random.seed(44)

"""
SquareGrid
anchor: the base point of the square
length: side length of the square
id: unique id
status: 0 or 1, for the grid is valid on map
cover: covered by a view: 0, no covering, n covered by n views
"""
class SquareGrid:
    def __init__(self, id, anchor, length):
        self.id = id
        self.anchor = anchor # bottom left position [width, height]
        self.length = length

    # check in a seed point is in grid
    def inGrid(self, pt):
        if pt[0] < self.anchor[0] or pt[0] > self.anchor[0]+self.length:
            return False
        if pt[1] < self.anchor[1] or pt[1] > self.anchor[1]+self.length:
            return False
        return True

    def center(self):
        return (self.anchor[0]+0.5*self.length, self.anchor[1]+0.5*self.length)

"""
GridMap
"""
class GridMap:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.grids = []
        self.validGrids = []

    # make a map with width of 90 meters and height of 60 meters
    def makeMap(self, width = 90, height = 60, res=1, sn=1000):
        self.height = height
        self.width = width
        # generate random seeds valid grid
        seeds = np.array([(random.randrange(width), random.randrange(height)) for c in range(sn)])
        # total number of grids
        nrow = int(self.height/res)
        ncol = int(self.width/res)
        # generate valid grid
        for i in range(nrow):
            for j in range(ncol):
                sg = SquareGrid(anchor=(j*res,i*res), length=res, id=j+i*ncol)
                self.grids.append(sg)
                if self.isValid(sg,seeds):
                    self.validGrids.append(sg)
        print("create map ({} meters x {} meters) with {} valid grids in total {}.".format(self.width, self.height, len(self.validGrids), len(self.grids)))
        return

    def grid(self,id):
        for grid in self.grids:
            if grid.id == id:
                return grid
        return None

    def isValid(self,grid,seeds):
        for pt in seeds:
            if grid.inGrid(pt):
                return True
        return False

    def loadMap(self,file):
        with open(file, 'r') as reader:
            lines = reader.read().splitlines()
            for i in range(len(lines)):
                data = lines[i].split(" ")
                if i == 0:
                    self.height = float(data[0])
                    self.width = float(data[1])
                else:
                    id = int(data[0])
                    anchor = [float(data[1]),float(data[2])]
                    length = float(data[3])
                    grid = SquareGrid(id, anchor,length)
                    self.grids.append(grid)
                    valid = int(data[4])
                    if valid == 1:
                        self.validGrids.append(grid)
        reader.close()
        print("load map ({} meters x {} meters) with {} valid grids in total {}.".format(self.width, self.height, len(self.validGrids), len(self.grids)))

    def saveMap(self,file):
        with open(file,'w') as writer:
            writer.write(str(self.height) + " " + str(self.width) + "\n")
            for grid in self.grids:
                line = str(grid.id) + " "\
                     + str(grid.anchor[0]) + " "\
                     + str(grid.anchor[1]) + " "\
                     + str(grid.length) + " "
                if grid in self.validGrids:
                    line += "1"
                else:
                    line += "0"
                line +="\n"
                writer.write(line)
            writer.close()

"""
Generate viewpoints
    map: the map with grid information
    fov: field of view of camera
    res: resolution
    dist: the distance from the ground to the camera
    type: "random": randomly generated the viewpoints, "uniform": each grid will have a viewpoint above on it.
"""
class ViewPointGenerator:
    def __init__(self, map=None, fov=(60.0,60.0), res=2.0, dist=3.0, type="uniform"):
        self.map = map
        self.fov = fov
        self.res = res
        self.dist = dist
        self.type = type

    def generateViewPoints(self):
        # generate a grid for viewpoints
        nrow = int(self.map.height/self.res)
        ncol = int(self.map.width/self.res)
        grids = []
        for i in range(nrow):
            for j in range(ncol):
                grids.append(SquareGrid(anchor=(j*self.res,i*self.res), length=self.res, id=j+i*ncol))

        # generate viewpoints either uniform distributed or randomly distributed
        vps = []
        for c in range(len(grids)):
            base = (0,0)
            if self.type == "random":
                base = (random.uniform(0,self.map.height), random.uniform(0,self.map.width))
            else:
                base = grids[c].center()

            # create a viewpoint with given distance and rotation angle 0.0
            vp = ViewPoint(id=c,px=base[0],py=base[1],pz=self.dist,ox=0,oy=0,oz=0,ow=1)
            vp.voxels = self.computeViewCover(vp)
            if len(vp.voxels) > 0:
                vps.append(vp)
        print("generate {} viewpoints with resolution {:.2f} meters at working distance {:.2f} meters.".format(len(vps), self.res, self.dist))
        return vps

    def computeViewCover(self, vp):
        """
        compute the covering grid under a viewpoint
        return a list of grid id
        """
        gridIds = set()
        xmin,xmax,ymin,ymax = self.coverBBox(vp)
        for grid in self.map.validGrids:
            if grid.anchor[0] > xmax or grid.anchor[0]+grid.length < xmin:
                continue
            if grid.anchor[1] > ymax or grid.anchor[1]+grid.length < ymin:
                continue
            gridIds.add(grid.id)
        return gridIds

    def coverBBox(self, vp):
        """
        cover bounding box is calculated with give the working distance (z) and the FOV
        return a rectangle vertices in np.array [x,y,0]
        """
        center = (vp.camera.position.x,vp.camera.position.y)
        fov1 = math.radians(0.5*self.fov[0])
        fov2 = math.radians(0.5*self.fov[1])
        xlen = vp.camera.position.z*np.tan(fov1)
        ylen = vp.camera.position.z*np.tan(fov2)
        xmin = center[0] - xlen
        xmax = center[0] + xlen
        ymin = center[1] - ylen
        ymax = center[1] + ylen
        return xmin,xmax,ymin,ymax

    def load(self,file):
        """
        load viewpoints from an external file
        """
        vps = []
        with open(file,'r') as reader:
            for line in reader.read().splitlines():
                data = line.split(" ")
                print(data)
                id = int(data[0])
                px = float(data[1])
                py = float(data[2])
                pz = float(data[3])
                ox = float(data[4])
                oy = float(data[5])
                oz = float(data[6])
                ow = float(data[7])
                vp = ViewPoint(id,px,py,pz,ox,oy,oz,ow)
                if len(data) > 8:
                    voxels = data[8:len(data)]
                    for v in voxels:
                        vp.voxels.add(int(v))
                vps.append(vp)
        reader.close()
        print("load {} viewpoints from {}.".format(len(vps), file))
        return vps

    def save(self,file,vps):
        """
        save viewpoints to an external file
        """
        with open(file, 'w') as writer:
            for vp in vps:
                line = str(vp.id) + " "\
                     + str(vp.camera.position.x) + " "\
                     + str(vp.camera.position.y) + " "\
                     + str(vp.camera.position.z) + " "\
                     + str(vp.camera.orientation.x) + " "\
                     + str(vp.camera.orientation.y) + " "\
                     + str(vp.camera.orientation.z) + " "\
                     + str(vp.camera.orientation.w)
                if not vp.voxels or len(vp.voxels) == 0:
                    line += "\n"
                else:
                    line += " "
                    i = 0
                    for v in vp.voxels:
                        if i == len(vp.voxels)-1:
                            line += str(v) + "\n"
                        else:
                            line += str(v) + " "
                            i += 1
                writer.write(line)
        writer.close()
        print("save viewpoints to file {}.".format(file))

"""
Plot helper
"""
class PlotHelper:
    def __init__(self,width,height,map):
        self.ax = self.createFig(width,height)
        self.map = map

    def createFig(self,width,height):
         fig = plt.figure(figsize=(width,height))
         return fig.add_subplot(111)

    def plotMap(self):
        self.ax.autoscale(enable=False)
        self.ax.set_xlim([-5,self.map.width+5])
        self.ax.set_ylim([-5,self.map.height+5])
        for grid in self.map.validGrids:
            patch = matplotlib.patches.Rectangle((grid.anchor),grid.length, grid.length, facecolor = "green", edgecolor='black',linewidth=1.0,alpha=0.2)
            self.ax.add_patch(patch)
        return

    def plotViewPoints(self,vps,type="coverage"):
        for vp in vps:
            self.plotViewPoint(vp,type)
        return

    def plotViewPoint(self,vp,type=None):
        if type == "coverage":
            for id in vp.voxels:
                grid = self.map.grid(id)
                patch = matplotlib.patches.Rectangle((grid.anchor),grid.length, grid.length, facecolor = "red", edgecolor='black',linewidth=1.0,alpha=0.2)
                self.ax.add_patch(patch)
        elif type == "point":
            x,y = vp.camera.position.x, vp.camera.position.y
            self.ax.scatter([x],[y], s=3, c='blue', marker='o')
        else:
            for id in vp.voxels:
                grid = self.map.grid(id)
                patch = matplotlib.patches.Rectangle((grid.anchor),grid.length, grid.length, facecolor = "red", edgecolor='black',linewidth=1.0,alpha=0.2)
                self.ax.add_patch(patch)
            x,y = vp.camera.position.x, vp.camera.position.y
            self.ax.scatter([x],[y], s=3, c='blue', marker='o')
        return

    def plotLines(self, vps, style='bo-',width=2, markersize=10, addArrow=False):
        X = [p.camera.position.x for p in vps]
        Y = [p.camera.position.y for p in vps]
        self.ax.plot(X,Y,style,linewidth=width,markersize=markersize)
        if addArrow:
            EX = [(vps[i+1].camera.position.x-vps[i].camera.position.x)/2 for i in range(len(vps)-1)]
            EY = [(vps[i+1].camera.position.y-vps[i].camera.position.y)/2 for i in range(len(vps)-1)]
            ax.quiver(X,Y,EX,EY,color='b', units='xy', scale=1, width=0.5)
        self.ax.axis('scaled')
        self.ax.axis('off')
        return

    def drawTrajectory(self, vps, speed=10, drawline=True, markEnd=False, insertText=False):
        # draw map
        self.plotMap()
        plt.draw()
        # draw start viewpoint
        start = vps[0]
        if markEnd:
            self.plotLines([start],'bh',markersize=16)
        else:
            self.plotLines([start],markersize=8)
        self.plotViewPoint(start)
        plt.draw()
        plt.pause(1) # puase 1 second to start
        # draw trajectory
        totalDist = 0
        info = self.ax.text(0,-2, "")
        for i in range(1, len(vps)):
            next = vps[i]
            if drawline:
                self.plotLines([start,next], markersize=8)
            else:
                self.plotLines([next], markersize=8)
            self.plotViewPoint(next)
            if insertText:
                info.set_position((0,-2))
                info.set_text("viewpoint index "+str(i+1)+ " of " + str(len(vps)))
            plt.draw()
            dist = vpDistance(start,next)
            totalDist += dist
            plt.pause(dist/speed)
            start = next
        if markEnd:
            self.plotLines([start],'bo',markersize=16)

        if insertText:
            info.set_position((0,-2))
            info.set_text("Done with {:.2f} meters traveling {} viewpoints.".format(totalDist,len(vps)))
        return

    def show(self):
        plt.show()
        return
