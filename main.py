import numpy as np 
from mpl_toolkits import mplot3d
from tkinter import *
import time 
import matplotlib.pyplot as plt 
import random as rd
import matplotlib.pylab as pl


MAX_speed = 20
MIN_speed = 5
WIDTH = 1000
HEIGHT = 800
N_AREAS_WIDTH = 5
N_AREAS_HEIGHT = 5
TOTALGRID = N_AREAS_HEIGHT*N_AREAS_WIDTH


MAX_SIZE = 10
VECTOR_SIZE = 30
LINE_SITE = 80
FIELD_OF_VIEW = 270

NBALLS = 50

COLORS = [ "pink", "blue", "green", "yellow", "purple", "orange", "white", "black" ]
SHOWGRIDCOLOR = False
SHOWGRIDS = True

TK = Tk()
CANVAS = Canvas(TK,width=WIDTH,height=HEIGHT)
"""
Defines a boid in space and time. This has all the functions that would affect a singular boid. 
"""

"""
NEED TO FIX THIS 
"""
def init_Boid():
    size = rd.randint(5,MAX_SIZE)
    px = rd.randint(0,WIDTH)
    py = rd.randint(0,HEIGHT)
    return [px-size,py-size,px+size,py+size],size

def seperateAreas():
    x_loc = np.linspace(0,WIDTH,N_AREAS_WIDTH+1)
    y_loc = np.linspace(0,HEIGHT,N_AREAS_HEIGHT+1)

    #Draws the gridlines
    if SHOWGRIDS:
        for y in range(N_AREAS_HEIGHT):
            CANVAS.create_line([0,y_loc[y]],[WIDTH,y_loc[y]])
        for x in range(N_AREAS_WIDTH):
            CANVAS.create_line([x_loc[x],0],[x_loc[x],HEIGHT])
    
    grid = [[x,y] for x in x_loc for y in y_loc]
    grid = np.reshape(grid,(N_AREAS_WIDTH+1,N_AREAS_HEIGHT+1,2))
    return grid

def gridGenerator(num):
    """
    Returns location in the shape of 0 2 
                                     1 3 
    """
    grid = seperateAreas()
    c = num%(N_AREAS_WIDTH)
    r = int(num/(N_AREAS_WIDTH))
    loc = np.concatenate((grid[c][r:r+2],grid[c+1][r:r+2]))
    return loc

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

def updateGrid(boidPos,space):
    """
    Position of the boid 
    """
    center = [(boidPos[0]+boidPos[2])/2,(boidPos[1]+boidPos[3])/2]
    if center[0]>WIDTH:
        center[0] = WIDTH
    if center[1]>HEIGHT:
        center[1] = HEIGHT
    grids = space.grids

    for g in grids: 
        if center[1]>=g.coord[0][1] and center[1]<=g.coord[1][1] and center[0]>=g.coord[1][0] and center[0]<=g.coord[3][0]:
            return g
        
#Caculates distances between two boids 
def calculateDistance(boid1,boid2):
    pos1 = boid1.pos
    pos2 = boid2.pos

    center1 = [(pos1[0]+pos1[2])/2,(pos1[3]+pos1[1])/2]
    center2 = [(pos2[0]+pos2[2])/2,(pos2[3]+pos2[1])/2]

    d = np.sqrt((center1[0]-center2[0])**2+(center1[1]-center2[1])**2)

    return d

class space: 
    def __init__(self,ballInterest = None):
        balls = []
        for i in range(NBALLS):
            Initpos, size = init_Boid()
            balls.append(Boid(Initpos,'blue',size,highVector=False))
            if ballInterest is not None:  
                if i ==ballInterest:
                    spec = Boid(Initpos,'green',size,highVector=True,highFOV=True)
                    balls.append(spec)     
        self.boids = balls
        self.width = WIDTH
        self.height = HEIGHT
        self.grids = [grid(i,self.boids) for i in range(TOTALGRID)]
        
class grid: 
    def __init__(self,num,boids):
        self.coord = gridGenerator(num)
        self.boids = findBoids(self.coord,boids) 
        self.num = num
        nums = list(map(lambda x : rd.randint(0,7), range(NBALLS)))
        self.color = COLORS[nums[rd.randint(0,7)]]

class Boid:
    #space = space()
    def __init__(self,pos,color,size,highVector=False, highFOV = False):
        """
        Initializes a boid object with the following parameters: 
            pos: the initial position of the boid is given by a box with (x1,y1) and (x2,y2), where the 
                 the first pair of coordinates represents the top left of the box while the second
                 represents the bottom right. pos has the form of [x1,y1,x2,y2]
            boid: Object drawn on the screen, for now it is a circle but will change to a triangle 
            vx: velocity in the x-direction
            vy: velocity in the y-direction 
            color: color of the boid 
            highlight: If want to focus on a specific boid, this will show its field of view. 
                       TO DO: Implement to show velocity vector
        """
        dr_theta = rd.randint(0,361)
        dr_r = rd.randint(-MAX_speed,MAX_speed+1)
        while np.abs(dr_r) < MIN_speed:
            dr_r = rd.randint(-MAX_speed,MAX_speed+1)


        self.pos = pos
        self.boid = CANVAS.create_oval(pos[0],pos[1],pos[2],pos[3],fill=color)
        self.vx = dr_r*np.cos(np.deg2rad(dr_theta))
        self.vy =  dr_r*np.sin(np.deg2rad(dr_theta))
        self.color = color
        self.size = size
        self.highVec = highVector
        self.highFOV = highFOV
        self.line = None
        self.fieldOfView = None
        self.alert = False 
        self.gridN = None
        self.mates = None   #friendly neighborhood boids that are within the field
                            #of view of this boid. 

    def move(self):
        """
        Moves the boid randomely on the canvas. If the boid hits the limit of the canvas, it will bounce. 
        TO DO: make it steer away from bounds and not just hit it.
        """
        #dr_theta = rd.randint(0,361)
        #dr_r = rd.randint(-MAX_speed,MAX_speed+1)
        
        #self.vx = self.vx + dr_r*np.cos(np.deg2rad(dr_theta))
        #self.vy = self.vy + dr_r*np.sin(np.deg2rad(dr_theta))
        
        #self.vx = dr_r*np.cos(np.deg2rad(dr_theta))
        #self.vy = dr_r*np.sin(np.deg2rad(dr_theta))

        CANVAS.move(self.boid,self.vx,self.vy)
        self.pos = CANVAS.coords(self.boid)

        if self.pos[3]>= HEIGHT or self.pos[1]<=0: 
            self.vy = self.flip(self.vy)
        if self.pos[2]>= WIDTH or self.pos[0]<=0:
            self.vx = self.flip(self.vx)
        
        self.pos = np.abs(self.pos)
        self.gridN = updateGrid(self.pos, s)

        if SHOWGRIDCOLOR:
            CANVAS.itemconfig(self.boid,fill = self.gridN.color)
        self.alertImpactWall()

        
        angles, lineAng = self.vision()

        if self.highVec:
            CANVAS.delete(self.line)
            self.line = CANVAS.create_line(self.pos[2] - self.size + VECTOR_SIZE*np.cos(lineAng),self.pos[3] - self.size + VECTOR_SIZE*np.sin(lineAng),
                        self.pos[0] + self.size - VECTOR_SIZE*np.cos(lineAng),self.pos[1] + self.size - VECTOR_SIZE*np.sin(lineAng),arrow='first')
            self.showDirectionVector(lineAng)
        
        if self.highFOV:
            CANVAS.delete(self.fieldOfView)
            self.fieldOfView = CANVAS.create_arc(self.pos[0] - self.size + LINE_SITE,self.pos[1] - self.size + LINE_SITE,
                        self.pos[2] + self.size - LINE_SITE,self.pos[3] + self.size - LINE_SITE,start = -np.rad2deg(angles[0])  , extent=FIELD_OF_VIEW)
            self.showFieldOfView(lineAng,angles)

    def showDirectionVector(self,lineAng):
        CANVAS.delete(self.line)
        CANVAS.move(self.line,self.vx,self.vy)
        self.line = CANVAS.create_line(self.pos[2] - self.size + VECTOR_SIZE*np.cos(lineAng),self.pos[3] - self.size + VECTOR_SIZE*np.sin(lineAng),
                        self.pos[0] + self.size - VECTOR_SIZE*np.cos(lineAng),self.pos[1] + self.size - VECTOR_SIZE*np.sin(lineAng),arrow='first')

    def showFieldOfView(self,lineAng,angles):
        CANVAS.delete(self.fieldOfView)

        CANVAS.move(self.fieldOfView,self.vx,self.vy)
        self.fieldOfView = CANVAS.create_arc(self.pos[0] - self.size + LINE_SITE, self.pos[1] - self.size + LINE_SITE,
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
            Input(s): Boid (object)
            Ouput(s): vision (float array) extreme angle of the field of view 
                      angle (float) directional angle of the boid object 
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
            CANVAS.itemconfig(self.boid,fill='red')
        elif self.pos[2]>= WIDTH-LINE_SITE or self.pos[0]<=0+LINE_SITE:
            self.alert = TRUE
            CANVAS.itemconfig(self.boid,fill='red')
        else:
            self.alert = False
            if SHOWGRIDCOLOR is False: 
                CANVAS.itemconfig(self.boid,fill='blue')

    def rule1(self,B,boids):
        """
        Implements the first rule of the boid project; each boid must avoid hitting
        each other.
            Input(s): boids object that are within a specific area of space.  
        """
        for b in boids: 
            d = calculateDistance(B,b)
            if d <= b.size + B.size: 
                return 0


        return 0 




    #def scanEnvironement():
    #def turn():

    #def alertBoidCollision(self):

ballInterest = 20
s=space(ballInterest=ballInterest)
balls = []


#for i in range(n):
#    Initpos, size = init_Boid()
#    balls.append(Boid(Initpos,'blue',size,highVector=False))
#    if i ==ballInterest:
#         spec = Boid(Initpos,'green',size,highVector=True,highFOV=True)
#         balls.append(spec)
            
#posBallInt = balls[ballInterest].pos

CANVAS.pack()  

while (True):
    [b.move() for b in s.boids]
    TK.update()
    time.sleep(0.09)

TK.mainloop()