import numpy as np 
from mpl_toolkits import mplot3d
from tkinter import *
import time 
import matplotlib.pyplot as plt 
import random as rd

MAX_speed = 1
WIDTH = 1000
HEIGHT = 500

"""
Defines a boid in space and time. This has all the functions that would affect a singular boid. 
"""

class Boid:
    def __init__(self,pos,color,highlight):
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
        self.highlight = highlight
        if highlight: 
            angles = self.vision()
            print (angles)
            self.fieldOfView = canvas.create_arc(pos[0]-50,pos[1]-50,pos[2]+50,pos[3]+50,
                start = np.rad2deg(angles[0]),extent=np.rad2deg(angles[1]))

    
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
        pos = canvas.coords(self.boid)
        
        if pos[3]>= HEIGHT or pos[1]<=0: 
            self.vy = self.flip(self.vy)
        if pos[2]>= WIDTH or pos[0]<=0:
            self.vx = self.flip(self.vx)

        if self.highlight:
            angles = self.vision()
            canvas.itemconfig(self.fieldOfView,start = np.rad2deg(angles[0]),extent=np.rad2deg(angles[1]))
            canvas.move(self.fieldOfView,self.vx,self.vy)
    
    def flip(self,v):
        """
        Flips a given boid's velocity (x or y)
        """
        return -v

    def vision(self):
        """
        Calculates the field of view of the boid. This is used to 
        """
        if self.vx == 0 and self.vy<0: 
            angle = -np.pi/2
        elif self.vx == 0 and self.vy>0:
            angle = np.pi/2 
        elif self.vx ==0 and self.vy == 0:
            angle = 0
        else: 
            angle = np.arctan2(self.vy,self.vx)
        st = angle - 2
        fn = angle + 2
        print (angle)

        if st < 0 and fn > 0:
            temp = st
            st = 2*np.pi - fn
            fn = 2*np.pi + temp  
        elif st < 0 or fn > 0: 
            temp = st
            st = fn 
            fn = temp

        vision = [st,fn] 
        return vision 

  
def init_Boid():
    size = rd.randint(5,25)
    px = rd.randint(0,WIDTH)
    py = rd.randint(0,HEIGHT)
    return [px-size,py-size,px+size,py+size]

tk = Tk()
canvas = Canvas(tk,width=1000,height=500)

nballs = 200
balls = []

ballInterest = 100

for i in range(nballs):
    balls.append(Boid(init_Boid(),'red',False))
    if i ==ballInterest:
         spec = Boid(init_Boid(),'green',True)
         balls.append(spec)
         
         
posBallInt = balls[ballInterest].pos

canvas.pack()  

while (True):
    for b in balls:
        b.move()
    tk.update()
    time.sleep(0.5)

tk.mainloop()