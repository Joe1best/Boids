import numpy as np 
from mpl_toolkits import mplot3d
from tkinter import *
import time 
import matplotlib.pyplot as plt 
import random as rd
import matplotlib.pylab as pl
import numpy.linalg as la
from multiprocessing import Pool

MAX_speed = 10
MIN_speed = 2
WIDTH = 1000
HEIGHT = 800
N_AREAS_WIDTH = 6
N_AREAS_HEIGHT = 6
TOTALGRID = N_AREAS_HEIGHT*N_AREAS_WIDTH

#Boid
MAX_SIZE = 20
MIN_SIZE = 15
VECTOR_SIZE = 30
LINE_SITE = 80
FIELD_OF_VIEW = 270
ANGLE_TOP = 40
ANGLE_BOTTOM = 70


#Axioms
_COLLISION_DISTANCE_ = LINE_SITE
_COLLISION_FACTOR_ = 7.5


NBALLS = 50
TOL = 10   #Make this as a function of total velocity 

COLORS = [ "pink", "cyan", "green", "yellow", "purple", "orange", "white", "black" ]
SHOWGRIDCOLOR = False
SHOWGRIDS = True

TK = Tk()
CANVAS = Canvas(TK,width=WIDTH,height=HEIGHT)
ballInterest = 2

def returnCanvas():
    return CANVAS

def init_Boid():
    size = rd.randint(MIN_SIZE,MAX_SIZE)
    px = rd.randint(size,WIDTH-size)   #Makes sure that the boid does not spawn outside the boundary
    py = rd.randint(size,HEIGHT-size)
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

def updateGrid(boid,space,size):
    """
    Position of the boid 
    """
    boidPos = boid.pos
    boidPos = check(boidPos)
    center = [(boidPos[0]+boidPos[2])/2,(boidPos[1]+boidPos[3])/2]
    grids = space.grids

    for g in grids: 
        if center[1]>=g.coord[0][1] and center[1]<=g.coord[1][1] and center[0]>=g.coord[1][0] and center[0]<=g.coord[3][0]:
            return g

def returnNeighborGrid(grid):
    """
    returns neighboring grids 
    """
    def neighbors8(grids): return [x for x in grids if x.num%N_AREAS_WIDTH!=0 and (x.num+1)%N_AREAS_WIDTH!=0 and int(x.num/N_AREAS_WIDTH)!=0 and int(x.num/N_AREAS_WIDTH)!=N_AREAS_HEIGHT-1]
    
    def neighbors3(grids): return [grids[0],grids[N_AREAS_WIDTH-1],grids[N_AREAS_WIDTH*(N_AREAS_HEIGHT-1)],grids[N_AREAS_HEIGHT*N_AREAS_WIDTH-1]]
    
    grids = s.grids
    grids = np.reshape(grids,(N_AREAS_WIDTH*N_AREAS_HEIGHT,))
    N8 = neighbors8(grids)
    N3 = neighbors3(grids)
    N5 = [x for x in grids if x not in N8 and x not in N3]
    neighborG = []
    if grid in N8:
        g = grid.num
        n_right = g + 1
        n_left = g - 1
        n_top = g - N_AREAS_WIDTH
        n_bottom = g + N_AREAS_WIDTH
        n_corner_1 = n_top + 1
        n_corner_2 = n_top - 1 
        n_corner_3 = n_bottom + 1
        n_corner_4 = n_bottom - 1
        neighborG.extend([grids[n_right],grids[n_left],grids[n_top],grids[n_bottom],grids[n_corner_1],
                        grids[n_corner_2], grids[n_corner_3],grids[n_corner_4]])
    if grid in N5:
        cond = int(grid.num/N_AREAS_WIDTH)
        cond_1 = grid.num%N_AREAS_WIDTH
        g = grid.num
        if cond==0:
            n_left = g - 1
            n_right = g + 1
            n_bottom = g + N_AREAS_WIDTH
            n_corner_1 = n_bottom - 1 
            n_corner_2 = n_bottom + 1 
            neighborG.extend([grids[n_left],grids[n_right],grids[n_bottom],grids[n_corner_1],grids[n_corner_2]])
        elif cond == N_AREAS_HEIGHT-1:
            n_left = g - 1
            n_right = g + 1
            n_top = g - N_AREAS_WIDTH
            n_corner_1 = n_top - 1 
            n_corner_2 = n_top + 1
            neighborG.extend([grids[n_left],grids[n_right],grids[n_top],grids[n_corner_1],grids[n_corner_2]])
        elif cond_1 == 0:
            n_right = g + 1
            n_top = g - N_AREAS_WIDTH
            n_bottom = g + N_AREAS_WIDTH
            n_corner_1 = n_top + 1 
            n_corner_2 = n_bottom + 1 
            neighborG.extend([grids[n_top],grids[n_right],grids[n_bottom],grids[n_corner_1],grids[n_corner_2]])
        elif cond_1 == N_AREAS_WIDTH-1:
            n_left = g - 1
            n_top = g - N_AREAS_WIDTH
            n_bottom = g + N_AREAS_WIDTH
            n_corner_1 = n_top - 1 
            n_corner_2 = n_bottom - 1
            neighborG.extend([grids[n_top],grids[n_left],grids[n_bottom],grids[n_corner_1],grids[n_corner_2]])
    
    if grid in N3:
        g = grid.num
        if grid is N3[0]:
            n_right = g+1
            n_bottom = g + N_AREAS_WIDTH
            n_corner_1 = n_bottom + 1 
            neighborG.extend([grids[n_right],grids[n_bottom],grids[n_corner_1]])
        elif grid is N3[1]:
            n_left = g - 1 
            n_bottom = g + N_AREAS_WIDTH
            n_corner_1 = n_bottom - 1
            neighborG.extend([grids[n_left],grids[n_bottom],grids[n_corner_1]])
        elif grid is N3[2]:
            n_top = g - N_AREAS_WIDTH
            n_right = g+1
            n_corner_1 = n_top + 1
            neighborG.extend([grids[n_top],grids[n_right],grids[n_corner_1]])
        elif grid is N3[3]:
            n_top = g - N_AREAS_WIDTH
            n_left = g-1
            n_corner_1 = n_top - 1
            neighborG.extend([grids[n_top],grids[n_left],grids[n_corner_1]])
        
    return neighborG
            
def check(boidpos):
    """
    Makes sure that the positions are within the bounds 
    """
    boidpos = np.asarray(boidpos)
    boidpos[boidpos<0] = 0 
    boidpos = list(boidpos)

    if boidpos[3]>HEIGHT:
        boidpos[3] = HEIGHT
    if boidpos[1]>HEIGHT:
        boidpos[1] = HEIGHT
    
    if boidpos[0] >WIDTH:
        boidpos[0] = WIDTH 
    if boidpos[2] > WIDTH:
        boidpos[2] = WIDTH 
    return boidpos

def limitSpeed(vector, max_magnitude, min_magnitude = 0.0):
    mag = magnitude(vector)
    if mag > max_magnitude:
        normalizing_factor = max_magnitude / mag
    elif mag < min_magnitude:
        normalizing_factor = min_magnitude / mag
    else: return vector

    return [value * normalizing_factor for value in vector]

def centerPos(boid):
    pos = boid.pos
    center = [(pos[0]+pos[2])/2,(pos[3]+pos[1])/2]
    return center

#Caculates distances between two boids 
def calculateDistance(boid1,boid2):
    center1 = centerPos(boid1)
    center2 = centerPos(boid2)

    d = np.sqrt((center1[0]-center2[0])**2+(center1[1]-center2[1])**2)

    return d

def magnitude(v1):
    """
    Takes the magnitude of vector v1
    """
    return np.sqrt(v1[0]**2+v1[1]**2)

def vectAng(v1,v2):
    """ 
    Returns the angle in radians between vectors 'v1' and 'v2'    
    """
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)


class drawBoid:
    def draw(self,b,angle,size):
        center = centerPos(b)  
        p1 = np.asarray([0,size/2])
        p2 = np.asarray([-size*np.cos(np.deg2rad(ANGLE_BOTTOM)),-size/2])
        p3 = np.asarray([size*np.cos(np.deg2rad(ANGLE_BOTTOM)),-size/2])
        
        p1,p2,p3 = self.rotate(p1,p2,p3,angle)
        
        p1 = p1 + center
        p2 = p2 + center 
        p3 = p3 + center

        points = np.reshape([p1,p2,p3],(6,))
        points = list(points)

        r = CANVAS.create_polygon(points,outline='black',fill='red',width=3)

        return r  
    
    def rotate(self,p1,p2,p3,theta):
        c, s = np.cos(theta), np.sin(theta)
        R = np.matrix([[c,-s],[s,c]])
        p1_ = np.dot(R,p1.T)
        p2_ = np.dot(R,p2.T)
        p3_ = np.dot(R,p3.T)

        return p1_,p2_,p3_

class space: 
    def __init__(self,ballInterest = None):
        balls = []
        for i in range(NBALLS):
            Initpos, size = init_Boid()
            balls.append(Boid(Initpos,i,'blue',size))
            
            #Loop below is to make sure that the particles are not spawning in inside each other
            #to begin with.
            j = 0
            if i !=0:
                while j < len(balls): 
                    d = calculateDistance(balls[-1],balls[j])
                    if d - (balls[-1].size+balls[j].size)<0 and balls[-1] != balls[j]:
                        CANVAS.delete(balls[-1].boid)
                        Initpos, size = init_Boid()
                        balls.pop()
                        balls.append(Boid(Initpos,i,'blue',size))
                        d = calculateDistance(balls[-1],balls[j])
                        j=-1
                    else: 
                        j= j +1
            
            #If there is a boid we want to observe 
            if ballInterest is not None:  
                if i ==ballInterest:
                    spec = Boid(Initpos,i,'green',size)
                    spec.special = True
                    balls.append(spec)     
        self.boids = balls
        self.width = WIDTH
        self.height = HEIGHT
        self.grids = [grid(i,self.boids) for i in range(TOTALGRID)]

    def updateSpace(self):
        [g.update(self.boids) for g in self.grids]
        
class grid: 
    def __init__(self,num,boids):
        self.coord = gridGenerator(num)
        self.boids = findBoids(self.coord,boids) 
        self.num = num
        nums = list(map(lambda x : rd.randint(0,7), range(NBALLS)))
        self.color = COLORS[nums[rd.randint(0,7)]]
    
    def update(self,boids):
        """
        Function that updates the boids that are within the grids everytime
        it is called
        """
        self.boids = findBoids(self.coord,boids)

class Boid:
    #space = space()
    def __init__(self,pos,num,color,size):
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
        self.num = num
        self.vx = dr_r*np.cos(np.deg2rad(dr_theta))
        self.vy =  dr_r*np.sin(np.deg2rad(dr_theta))
        self.color = color
        self.size = size
        self.boid = None
        self.line = None
        self.fieldOfView = None
        self.alert = False 
        self.gridN = None
        self.mates = None   
        self.special = False
    
    def bounceWall(self):
        if self.pos[3]>= HEIGHT or self.pos[1]<=0: 
            self.vy = self.flip(self.vy)
        if self.pos[2]>= WIDTH or self.pos[0]<=0:
            
            self.vx = self.flip(self.vx)
            if self.special:
                print (self.vx)
    def move(self):
        """
        Moves the boid randomely on the canvas. If the boid hits the limit of the canvas, it will bounce. 
        TO DO: make it steer away from bounds and not just hit it.
        """

        self.gridN = updateGrid(self,s,self.size)
        self.mates = self.findMates()
        dvx, dvy = self.rule1(self.mates)

        self.vx += dvx
        self.vy += dvy 

        vs = limitSpeed([self.vx,self.vy],MAX_speed,MIN_speed)
        self.vx = vs[0]
        self.vy = vs[1]

        self.bounceWall()

        angles, lineAng = self.vision()
        
        CANVAS.delete(self.boid)
        self.boid = drawBoid().draw(self,lineAng,self.size)
        if self.special:
            print (self.pos)
            print (self.vx,self.vy)
       
        CANVAS.move(self.boid,self.vx,self.vy)
        CANVAS.delete(self.boid)
        self.boid = drawBoid().draw(self,lineAng,self.size)
        
        coord = CANVAS.coords(self.boid)
        if self.special:
            print (coord)
        x_offset = self.size*np.cos(np.deg2rad(ANGLE_BOTTOM))
        

        self.pos = [coord[0]-x_offset,coord[1],coord[4],coord[5]]
        if self.special:
            print (self.pos,'\n')
        
        angles, lineAng = self.vision()

        if SHOWGRIDCOLOR:
            CANVAS.itemconfig(self.boid,fill = self.gridN.color)
        self.alertImpactWall()


        if self.special:
            CANVAS.delete(self.line)
            self.line = CANVAS.create_line(self.pos[2] - self.size + VECTOR_SIZE*np.cos(lineAng),self.pos[3] - self.size + VECTOR_SIZE*np.sin(lineAng),
                        self.pos[0] + self.size - VECTOR_SIZE*np.cos(lineAng),self.pos[1] + self.size - VECTOR_SIZE*np.sin(lineAng),arrow='first')
            self.showDirectionVector(lineAng)
        
        if self.special:
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
                      angle (float) angle at which the boid is pointing
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

    def findMates(self):
        """
        Finds mates that are within the field of view of the boid
            Input(s): boid (object)
            Output(s): mates (array of objects)  
        """
        mates = []
        potentialColl = []
        nGrid = returnNeighborGrid(self.gridN)
        nGrid.extend([self.gridN])
        for g in nGrid:
            potentialColl.extend(g.boids)
        limitAng, angle = self.vision()
        dirVec = [np.cos(angle),np.sin(angle)]
        p = self.pos
        center = [(p[0]+p[2])/2,(p[1]+p[3])/2]
        for p in potentialColl:
            #Makes sure that we are not comparing the boid with itself
            if p == self:
                continue
            posM = p.pos
            centerM = [(posM[0]+posM[2])/2,(posM[1]+posM[3])/2]
            angleFromSelf = np.arctan2(center[1]-centerM[1],center[0]-centerM[0])
            vecFromSelf = [np.cos(angleFromSelf),np.sin(angleFromSelf)]
            if (posM[0]-center[0])**2+(posM[1]-center[1])**2 <=LINE_SITE**2 or (posM[2]-center[0])**2+(posM[3]-center[1])**2 <=LINE_SITE**2:
                if vectAng(dirVec,vecFromSelf) < FIELD_OF_VIEW/2 : 
                    mates.append(p)
        return mates


    def rule1(self,mates):
        """
        Implements the first rule of the boid project; each boid must avoid hitting
        each other. This is done by repelling them with a force that is proportionel to the
        distance between boids and inversely proportionel to the distance squared. This force 
        will then be multiplied by some constant determined "experimentaly" to find the perfect
        and most natural boid avoidance. 
            Input(s): boids object that are within a specific area of space (mates)  
        """
        c = [0,0]
        for b in mates:
            d = calculateDistance(self,b)
            if d <=_COLLISION_DISTANCE_ and b != self and d !=0:
                centerM = centerPos(b)
                center = centerPos(self)
                diff = [centerM[0]-center[0],centerM[1]-center[1]]
                magnitudeDiff = magnitude(diff) - self.size - b.size
                c[0] = c[0] - diff[0]/magnitudeDiff**2*_COLLISION_FACTOR_
                c[1] = c[1] - diff[1]/magnitudeDiff**2*_COLLISION_FACTOR_
        return c 

    def rule2(self,mates):
        """
        Implements rule 2 of the boid algo. This rule is in charged of handling the 
        alignment of nearby boids. 
        """
        return 0

    #def scanEnvironement():
    #def turn():

    #def alertBoidCollision(self):

s=space(ballInterest=ballInterest)

def run():
    CANVAS.pack()
    p = Pool(8)
    while (True):
        s.updateSpace()
        #[print (g.boids) for g in s.grids if g.num==1]
        boids = s.boids
        nullVar = p.map(lambda boi: boi.move(), boids)
        #[b.move() for b in s.boids]
        TK.update()
        time.sleep(0.05)
    TK.mainloop()

if __name__ == '__main__':
    #run()
    CANVAS.pack()
    #p = Pool(8)
    while (True):
        s.updateSpace()
        #[print (g.boids) for g in s.grids if g.num==1]
        #boids = s.boids
        #f = lambda boi: boi.move()
        #nullVar = p.map(f, boids)
        [b.move() for b in s.boids]
        TK.update()
        time.sleep(0.05)
    TK.mainloop()