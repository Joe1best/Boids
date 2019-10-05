import numpy as np 
from mpl_toolkits import mplot3d
from tkinter import *
import time 
import matplotlib.pyplot as plt 
import random as rd

MAX_speed = 1
WIDTH = 1000
HEIGHT = 500

class Boid:
    def __init__(self,pos,color,highlight):
        self.pos = pos
        self.boid = canvas.create_oval(pos[0],pos[1],pos[2],pos[3],fill=color)
        self.vx = 0
        self.vy = 0
        self.color = color
        self.highlight = highlight
    
    def move(self):
        dr_theta = rd.randint(0,361)
        dr_r = rd.randint(-MAX_speed,MAX_speed+1)
        
        self.vx = self.vx + dr_r*np.cos(np.deg2rad(dr_theta))
        self.vy = self.vy + dr_r*np.sin(np.deg2rad(dr_theta))
        
        canvas.move(self.boid,self.vx,self.vy)
        pos = canvas.coords(self.boid)
        
        if pos[3]>= HEIGHT or pos[1]<=0: 
            self.vy = self.flip(self.vy)
        if pos[2]>= WIDTH or pos[0]<=0:
            self.vx = self.flip(b.vx)
        if self.highlight:
            angles = self.vision()
            print (angles)
            canvas.create_arc(20*np.cos(angles[0]),20*np.sin(angles[1]),20*np.cos(angles[1]),20*np.sin(angles[1]),start = np.rad2deg(angles[0]),extent=np.rad2deg(angles[1]),fill='green')

            
    def flip(self,v):
        return -v

    def vision(self):
        if self.vx == 0 and self.vy<0: 
            angle = -np.pi/2
        elif self.vx == 0 and self.vy>0:
            angle = np.pi/2 
        elif self.vx ==0 and self.vy == 0:
            angle = 0
        else: 
            angle = np.arctan(self.vy/self.vx)
        vision = [angle-2,angle+2] 
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