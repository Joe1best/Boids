import numpy as np 
from mpl_toolkits import mplot3d
from tkinter import *
import time 
import matplotlib.pyplot as plt 
import random as rd

MAX_speed = 2
WIDTH = 1000
HEIGHT = 800

MAX_SIZE = 10
VECTOR_SIZE = 30
LINE_SITE = 40
FIELD_OF_VIEW = 270

"""
Defines a boid in space and time. This has all the functions that would affect a singular boid. 
"""
def findBoids(area,boids):
    """
    Function that returns boid objects that are within the specified area. 
    """
    return 0

class Boid:
    def __init__(self,pos,color,size,highVector=False, highFOV = False):
        """
        Initializes a boid object with the following parameters: 
            pos: the position of the boid is given by a box with (x1,y1) and (x2,y2), where the 
                 the first pair of coordinates represents the top left of the box while the second
                 represents the bottom right. pos has the form of [x1,y1,x2,y2]
            boid: Object drawn on the screen, for now it is a circle but will change to a triangle 
            vx: velocity in the x-direction
            vy: velocity in the y-direction 
            color: color of the boid 
            highlight: If want to focus on a specific boid, this will show its field of view. 
                       TO DO: Implement to show velocity vector
        """
        self.pos = pos
        self.boid = canvas.create_oval(pos[0],pos[1],pos[2],pos[3],fill=color)
        self.vx = 0
        self.vy = 0
        self.color = color
        self.size = size
        self.highVec = highVector
        self.highFOV = highFOV
        self.line =None
        self.fieldOfView = None
        self.alert = False
            
    def move(self):
        """
        Moves the boid randomely on the canvas. If the boid hits the limit of the canvas, it will bounce. 
        TO DO: make it steer away from bounds and not just hit it.
        """
        dr_theta = rd.randint(0,361)
        dr_r = rd.randint(-MAX_speed,MAX_speed+1)
        
        self.vx = self.vx + dr_r*np.cos(np.deg2rad(dr_theta))
        self.vy = self.vy + dr_r*np.sin(np.deg2rad(dr_theta))
        
        canvas.move(self.boid,self.vx,self.vy)
        self.pos = canvas.coords(self.boid)

        self.alertImpactWall()

        if self.pos[3]>= HEIGHT or self.pos[1]<=0: 
            self.vy = self.flip(self.vy)
        if self.pos[2]>= WIDTH or self.pos[0]<=0:
            self.vx = self.flip(self.vx)
        angles, lineAng = self.vision()

        if self.highVec:
            canvas.delete(self.line)
            self.line = canvas.create_line(self.pos[2] - self.size + VECTOR_SIZE*np.cos(lineAng),self.pos[3] - self.size + VECTOR_SIZE*np.sin(lineAng),
                        self.pos[0] + self.size - VECTOR_SIZE*np.cos(lineAng),self.pos[1] + self.size - VECTOR_SIZE*np.sin(lineAng),arrow='first')
            self.showDirectionVector(lineAng)
        
        if self.highFOV:
            canvas.delete(self.fieldOfView)
            self.fieldOfView = canvas.create_arc(self.pos[0] - self.size + LINE_SITE,self.pos[1] - self.size + LINE_SITE,
                        self.pos[2] + self.size - LINE_SITE,self.pos[3] + self.size - LINE_SITE,start = -np.rad2deg(angles[0])  , extent=FIELD_OF_VIEW)
            self.showFieldOfView(lineAng,angles)

    def showDirectionVector(self,lineAng):
        canvas.delete(self.line)
        canvas.move(self.line,self.vx,self.vy)
        self.line = canvas.create_line(self.pos[2] - self.size + VECTOR_SIZE*np.cos(lineAng),self.pos[3] - self.size + VECTOR_SIZE*np.sin(lineAng),
                        self.pos[0] + self.size - VECTOR_SIZE*np.cos(lineAng),self.pos[1] + self.size - VECTOR_SIZE*np.sin(lineAng),arrow='first')

    def showFieldOfView(self,lineAng,angles):
        canvas.delete(self.fieldOfView)

        canvas.move(self.fieldOfView,self.vx,self.vy)
        self.fieldOfView = canvas.create_arc(self.pos[0] - self.size + LINE_SITE, self.pos[1] - self.size + LINE_SITE,
                        self.pos[2] + self.size - LINE_SITE,self.pos[3] + self.size - LINE_SITE, start = -np.rad2deg(angles[0]),extent=FIELD_OF_VIEW)

    def flip(self,v):
        """
        Flips a given boid's velocity (x or y)
        """
        return -v

    def vision(self):
        """
        Calculates the field of view of the boid. This is used to figure out how many boids are in the vicinity 
        so it can avoid them.
        """
        if self.vx == 0 and self.vy<0: 
            angle = 3*np.pi/2
        elif self.vx == 0 and self.vy>0:
            angle = np.pi/2 
        elif self.vx ==0 and self.vy == 0:
            angle = 0
        else: 
            angle = np.arctan2(self.vy,self.vx)
        st = angle - np.deg2rad(FIELD_OF_VIEW/2)
        if st < 0: 
            st = 2*np.pi + st
        vision = [st-np.pi/2,0] 
        return vision,angle

    def alertImpactWall(self):
        if self.pos[3]>= HEIGHT-LINE_SITE or self.pos[1]<=0+LINE_SITE: 
            self.alert = TRUE
            canvas.itemconfig(self.boid,fill='red')
        elif self.pos[2]>= WIDTH-LINE_SITE or self.pos[0]<=0+LINE_SITE:
            self.alert = TRUE
            canvas.itemconfig(self.boid,fill='red')
        else:
            self.alert = False
            canvas.itemconfig(self.boid,fill='blue')

    #def turn():

    #def alertBoidCollision(self):


def init_Boid():
    size = rd.randint(5,MAX_SIZE)
    px = rd.randint(0,WIDTH)
    py = rd.randint(0,HEIGHT)
    return [px-size,py-size,px+size,py+size],size

tk = Tk()
canvas = Canvas(tk,width=WIDTH,height=HEIGHT)

nballs = 100
balls = []

ballInterest = 99

for i in range(nballs):
    Initpos, size = init_Boid()
    balls.append(Boid(Initpos,'blue',size,highVector=False))
    if i ==ballInterest:
         spec = Boid(Initpos,'green',size,highVector=True,highFOV=True)
         balls.append(spec)
            
posBallInt = balls[ballInterest].pos

canvas.pack()  

while (True):
    [b.move() for b in balls]
    tk.update()
    time.sleep(0.09)

tk.mainloop()