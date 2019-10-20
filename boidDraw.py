import numpy as np 
from mpl_toolkits import mplot3d
from tkinter import *
from main import CANVAS

ANGLE_TOP = 40
ANGLE_BOTTOM = 70
SIZE = 25

class drawBoid:
    def __init__(self,b,angle):
        """
        size corresponds to the height of the triangle
        boid is the boid class defined in the "main" file
        """
        self.size = b.size
        self.object = b
        self.drawing = self.draw(b,angle)

    def draw(self,b,angle):
        pos = CANVAS.coords(b.boid)
        #CANVAS.delete(b.boid)
        center = centerPos(b)  
        p1 = np.asarray([0,SIZE/2])
        p2 = np.asarray([-self.size*np.cos(np.deg2rad(ANGLE_BOTTOM)),-self.size/2])
        p3 = np.asarray([self.size*np.cos(np.deg2rad(ANGLE_BOTTOM)),-self.size/2])
        
        p1,p2,p3 = self.rotate(p1,p2,p3,np.angle)
        
        p1 = p1 + center
        p2 = p2 + center 
        p3 = p3 + center
        
        
        print (p1,p2,p3)
        points = np.reshape([p1,p2,p3],(6,))
        points = list(points)

        r = CANVAS.create_polygon(points,outline='black',fill='red',width=3)

        return r  
    
    def rotate(self,p1,p2,p3,theta):
        c, s = np.cos(theta), np.sin(theta)
        R = np.matrix([[c,-s],[s,c]])
        print (R)
        p1_ = np.dot(R,p1.T)
        p2_ = np.dot(R,p2.T)
        p3_ = np.dot(R,p3.T)

        return p1_,p2_,p3_


