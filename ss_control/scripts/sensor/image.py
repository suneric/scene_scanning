#!/usr/bin/env python3
import numpy as np
import cv2 as cv

class DepthImage:
    def __init__(self, depth):
        self.depth = depth

    def center(self):
        h,w = self.depth.shape
        return w/2,h/2

    def show(self):
        cv.namedWindow("depth")
        cv.imshow("depth", self.depth)
        cv.waitKey(0) #& 0xFF

    def normal(self):
        return None

    # calculate mean distance in a small pixel frame around u,v
    # a non-zero mean value for the pixel with its neighboring pixels
    def distance(self,u,v,size=3):
        dist_list=[]
        for i in range(-size,size):
            for j in range(-size,size):
                value = self.depth[int(v+j),int(u+i)]
                if value > 0.0:
                    dist_list.append(value)
        if not dist_list:
            return -1
        else:
            return np.mean(dist_list)

    def normal(self, u,v, size=3):
        dzdu = (self.distance(u+size,v)-self.distance(u-size,v))/(2*size)
        dzdv = (self.distance(u,v+size)-self.distance(u,v-size))/(2*size)
        n = np.array([-dzdu, -dzdv, 1])
        unit_n = n/np.linalg.norm(n)
        return unit_n

    def _draw_normals(self, size=3):
        if not self.depth.shape:
            return None
        else:
            img = self.depth
            w,h = img.shape
            print(w,h)
            normals = np.zeros((w,h,3),dtype=np.uint8)
            for y in range(size, w-size):
                for x in range(size, h-size):
                    normals[x,y] = self.normal(x,y,size)
            return normals

class ColorImage:
    def __init__(self, color):
        self.color = color

    def center(self):
        h,w,c = self.color.shape
        return w/2, h/2

    def show(self,dist):
        cx,cy = self.center()
        if dist <= 1.0:
            self._draw_box(20,cx,cy,(0,0,255),1,dist) # color (b,g,r)
        else:
            self._draw_box(20,cx,cy,(0,255,0),1,dist) # color (b,g,r)

        cv.namedWindow("color")
        cv.imshow("color", self.color)
        cv.waitKey(3) #& 0xFF

    def _draw_box(self,size,cx,cy,clr,lw,dist):
        cv.rectangle(self.color, (int(cx-size), int(cy-size)), (int(cx+size), int(cy+size)), clr, lw)
        if dist != None:
            strDist = "{:.6}".format(str(dist)) # cm
            font_scale = 0.3
            cv.putText(self.color, strDist, (int(cx-size+4), int(cy)), cv.FONT_HERSHEY_SIMPLEX, font_scale, clr, lw)
