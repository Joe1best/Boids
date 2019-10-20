import numpy as np 
from tkinter import *
import random as rd
import init

def init_Boid():
    size = rd.randint(init.MIN_SIZE,init.MAX_SIZE)
    px = rd.randint(size,init.WIDTH-size)   #Makes sure that the boid does not spawn outside the boundary
    py = rd.randint(size,init.HEIGHT-size)
    return [px-size,py-size,px+size,py+size],size

def findBoids(area,boids):
    """
    Function that returns boid objects that are within the specified area. 
    Format of area is the grid coordinates
    """
    neighboordBoid = []
    for b in boids: 
        pos = b.pos
        center = [(pos[0]+pos[2])/2,(pos[3]+pos[1])/2]
        if center[1]>=area[0][1] and center[1]<=area[1][1] and center[0]>=area[1][0] and center[0]<=area[3][0]:
            neighboordBoid.append(b)
    return neighboordBoid 

def check(boidpos):
    """
    Makes sure that the positions are within the bounds 
    """
    boidpos = np.asarray(boidpos)
    boidpos[boidpos<0] = 0 
    boidpos = list(boidpos)

    if boidpos[3]>init.HEIGHT:
        boidpos[3] = init.HEIGHT
    if boidpos[1]>init.HEIGHT:
        boidpos[1] = init.HEIGHT
    
    if boidpos[0] >init.WIDTH:
        boidpos[0] = init.WIDTH 
    if boidpos[2] > init.WIDTH:
        boidpos[2] = init.WIDTH 
    return boidpos

def centerPos(boid):
    pos = boid.pos
    center = [(pos[0]+pos[2])/2,(pos[3]+pos[1])/2]
    return center