import numpy as np 
from tkinter import *
import time 
import random as rd
import numpy.linalg as la
import init
import boidUtil
import random as rd

ballInterest = 2

def seperateAreas():
    x_loc = np.linspace(0,init.WIDTH,init.N_AREAS_WIDTH+1)
    y_loc = np.linspace(0,init.HEIGHT,init.N_AREAS_WIDTH+1)

    #Draws the gridlines
    if init.SHOWGRIDS:
        for y in range(init.N_AREAS_HEIGHT):
            init.CANVAS.create_line([0,y_loc[y]],[init.WIDTH,y_loc[y]])
        for x in range(init.N_AREAS_WIDTH):
            init.CANVAS.create_line([x_loc[x],0],[x_loc[x],init.HEIGHT])
    
    grid = [[x,y] for x in x_loc for y in y_loc]
    grid = np.reshape(grid,(init.N_AREAS_WIDTH+1,init.N_AREAS_HEIGHT+1,2))
    return grid

def gridGenerator(num):
    """
    Returns location in the shape of 0 2 
                                     1 3 
    """
    grid = seperateAreas()
    c = num%(init.N_AREAS_WIDTH)
    r = int(num/(init.N_AREAS_WIDTH))
    loc = np.concatenate((grid[c][r:r+2],grid[c+1][r:r+2]))
    return loc
 
def updateGrid(boid,space,size):
    """
    Position of the boid 
    """
    boidPos = boid.pos
    boidPos = boidUtil.check(boidPos)
    center = [(boidPos[0]+boidPos[2])/2,(boidPos[1]+boidPos[3])/2]
    grids = space.grids

    for g in grids: 
        if center[1]>=g.coord[0][1] and center[1]<=g.coord[1][1] and center[0]>=g.coord[1][0] and center[0]<=g.coord[3][0]:
            return g

def returnNeighborGrid(grid):
    """
    returns neighboring grids 
    """
    def neighbors8(grids): return [x for x in grids if x.num%init.N_AREAS_WIDTH!=0 and (x.num+1)%init.N_AREAS_WIDTH!=0 and int(x.num/init.N_AREAS_WIDTH)!=0 and int(x.num/init.N_AREAS_WIDTH)!=init.N_AREAS_WIDTH-1]
    
    def neighbors3(grids): return [grids[0],grids[init.N_AREAS_WIDTH-1],grids[init.N_AREAS_WIDTH*(init.N_AREAS_WIDTH-1)],grids[init.N_AREAS_WIDTH*init.N_AREAS_WIDTH-1]]
    
    grids = s.grids
    grids = np.reshape(grids,(init.N_AREAS_WIDTH*init.N_AREAS_WIDTH,))
    N8 = neighbors8(grids)
    N3 = neighbors3(grids)
    N5 = [x for x in grids if x not in N8 and x not in N3]
    neighborG = []
    if grid in N8:
        g = grid.num
        n_right = g + 1
        n_left = g - 1
        n_top = g - init.N_AREAS_WIDTH
        n_bottom = g + init.N_AREAS_WIDTH
        n_corner_1 = n_top + 1
        n_corner_2 = n_top - 1 
        n_corner_3 = n_bottom + 1
        n_corner_4 = n_bottom - 1
        neighborG.extend([grids[n_right],grids[n_left],grids[n_top],grids[n_bottom],grids[n_corner_1],
                        grids[n_corner_2], grids[n_corner_3],grids[n_corner_4]])
    if grid in N5:
        cond = int(grid.num/init.N_AREAS_WIDTH)
        cond_1 = grid.num%init.N_AREAS_WIDTH
        g = grid.num
        if cond==0:
            n_left = g - 1
            n_right = g + 1
            n_bottom = g + init.N_AREAS_WIDTH
            n_corner_1 = n_bottom - 1 
            n_corner_2 = n_bottom + 1 
            neighborG.extend([grids[n_left],grids[n_right],grids[n_bottom],grids[n_corner_1],grids[n_corner_2]])
        elif cond == init.N_AREAS_WIDTH-1:
            n_left = g - 1
            n_right = g + 1
            n_top = g - init.N_AREAS_WIDTH
            n_corner_1 = n_top - 1 
            n_corner_2 = n_top + 1
            neighborG.extend([grids[n_left],grids[n_right],grids[n_top],grids[n_corner_1],grids[n_corner_2]])
        elif cond_1 == 0:
            n_right = g + 1
            n_top = g - init.N_AREAS_WIDTH
            n_bottom = g + init.N_AREAS_WIDTH
            n_corner_1 = n_top + 1 
            n_corner_2 = n_bottom + 1 
            neighborG.extend([grids[n_top],grids[n_right],grids[n_bottom],grids[n_corner_1],grids[n_corner_2]])
        elif cond_1 == init.N_AREAS_WIDTH-1:
            n_left = g - 1
            n_top = g - init.N_AREAS_WIDTH
            n_bottom = g + init.N_AREAS_WIDTH
            n_corner_1 = n_top - 1 
            n_corner_2 = n_bottom - 1
            neighborG.extend([grids[n_top],grids[n_left],grids[n_bottom],grids[n_corner_1],grids[n_corner_2]])
    
    if grid in N3:
        g = grid.num
        if grid is N3[0]:
            n_right = g+1
            n_bottom = g + init.N_AREAS_WIDTH
            n_corner_1 = n_bottom + 1 
            neighborG.extend([grids[n_right],grids[n_bottom],grids[n_corner_1]])
        elif grid is N3[1]:
            n_left = g - 1 
            n_bottom = g + init.N_AREAS_WIDTH
            n_corner_1 = n_bottom - 1
            neighborG.extend([grids[n_left],grids[n_bottom],grids[n_corner_1]])
        elif grid is N3[2]:
            n_top = g - init.N_AREAS_WIDTH
            n_right = g+1
            n_corner_1 = n_top + 1
            neighborG.extend([grids[n_top],grids[n_right],grids[n_corner_1]])
        elif grid is N3[3]:
            n_top = g - init.N_AREAS_WIDTH
            n_left = g-1
            n_corner_1 = n_top - 1
            neighborG.extend([grids[n_top],grids[n_left],grids[n_corner_1]])
        
    return neighborG
            
def limitSpeed(vector, max_magnitude, min_magnitude = 0.0):
    mag = magnitude(vector)
    if mag > max_magnitude:
        normalizing_factor = max_magnitude / mag
    elif mag < min_magnitude:
        normalizing_factor = min_magnitude / mag
    else: return vector

    return [value * normalizing_factor for value in vector]

#Caculates distances between two boids 
def calculateDistance(boid1,boid2):
    center1 = boidUtil.centerPos(boid1)
    center2 = boidUtil.centerPos(boid2)

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
    def draw(self,b,angle,size,color):
        center = boidUtil.centerPos(b)  
        p1 = np.asarray([0,size/2])
        p2 = np.asarray([-size*np.cos(np.deg2rad(init.ANGLE_BOTTOM)),-size/2])
        p3 = np.asarray([size*np.cos(np.deg2rad(init.ANGLE_BOTTOM)),-size/2])
        
        p1,p2,p3 = self.rotate(p1,p2,p3,angle)
        
        p1 = p1 + center
        p2 = p2 + center 
        p3 = p3 + center

        points = np.reshape([p1,p2,p3],(6,))
        points = list(points)

        r = init.CANVAS.create_polygon(points,outline='black',fill=color,width=2)

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
        nums = list(map(lambda x : rd.randint(0,len(init.COLORS)-1), range(init.NBALLS)))
        for i in range(init.NBALLS):
            Initpos, size = boidUtil.init_Boid()
            balls.append(Boid(Initpos,i,init.COLORS[nums[i]],size))
            
            #Loop below is to make sure that the particles are not spawning in inside each other
            #to begin with.
            j = 0
            if i !=0:
                while j < len(balls): 
                    d = calculateDistance(balls[-1],balls[j])
                    if d - (balls[-1].size+balls[j].size)<0 and balls[-1] != balls[j]:
                        init.CANVAS.delete(balls[-1].boid)
                        Initpos, size = boidUtil.init_Boid()
                        balls.pop()
                        balls.append(Boid(Initpos,i,init.COLORS[nums[i]],size))
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
        self.width = init.WIDTH
        self.height = init.HEIGHT
        self.grids = [grid(i,self.boids) for i in range(init.TOTALGRID)]

    def updateSpace(self):
        [g.update(self.boids) for g in self.grids]
        
class grid: 
    def __init__(self,num,boids):
        self.coord = gridGenerator(num)
        self.boids = boidUtil.findBoids(self.coord,boids) 
        self.num = num
        nums = list(map(lambda x : rd.randint(0,7), range(init.NBALLS)))
        self.color = init.COLORS[nums[rd.randint(0,7)]]
    
    def update(self,boids):
        """
        Function that updates the boids that are within the grids everytime
        it is called
        """
        self.boids = boidUtil.findBoids(self.coord,boids)

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
        dr_r = rd.randint(-init.MAX_SPEED,init.MAX_SPEED+1)
        while np.abs(dr_r) < init.MIN_SPEED:
            dr_r = rd.randint(-init.MAX_SPEED,init.MAX_SPEED+1)
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
        if self.pos[3]>= init.HEIGHT or self.pos[1]<=0: 
            self.vy = self.flip(self.vy)
        if self.pos[2]>= init.WIDTH or self.pos[0]<=0:
            self.vx = self.flip(self.vx)
         
    def move(self):
        """
        Moves the boid randomely on the canvas. If the boid hits the limit of the canvas, it will bounce. 
        TO DO: make it steer away from bounds and not just hit it.
        """
        self.gridN = updateGrid(self,s,self.size)
        self.mates = self.findMates()
        dvx_1, dvy_1 = self.rule1(self.mates)
        dvx_2,dvy_2 = self.rule2(self.mates)
        dvx_3,dvy_3 = self.rule3(self.mates)

        self.vx += dvx_1+dvx_2+dvx_3
        self.vy += dvy_1+dvy_2+dvy_3

        vs = limitSpeed([self.vx,self.vy],init.MAX_SPEED,init.MIN_SPEED)
        self.vx = vs[0]
        self.vy = vs[1]

        self.bounceWall()

        angles, lineAng = self.vision()
        
        init.CANVAS.delete(self.boid)
        self.boid = drawBoid().draw(self,lineAng-np.pi/2,self.size,self.color)

        init.CANVAS.move(self.boid,self.vx,self.vy)
        init.CANVAS.delete(self.boid)
        self.boid = drawBoid().draw(self,lineAng-np.pi/2,self.size,self.color)
        
        self.pos = [self.pos[0]+self.vx,self.pos[1]+self.vy,self.pos[2]+self.vx,self.pos[3]+self.vy]
      
        angles, lineAng = self.vision()

        if init.SHOWGRIDCOLOR:
            init.CANVAS.itemconfig(self.boid,fill = self.gridN.color)
        self.alertImpactWall()

        if self.special:
            init.CANVAS.delete(self.line)
            self.line = init.CANVAS.create_line(self.pos[2] - self.size + init.VECTOR_SIZE*np.cos(lineAng),self.pos[3] - self.size + init.VECTOR_SIZE*np.sin(lineAng),
                        self.pos[0] + self.size - init.VECTOR_SIZE*np.cos(lineAng),self.pos[1] + self.size - init.VECTOR_SIZE*np.sin(lineAng),arrow='first')
            self.showDirectionVector(lineAng)
        
        if self.special:
            init.CANVAS.delete(self.fieldOfView)
            self.fieldOfView = init.CANVAS.create_arc(self.pos[0] - self.size + init.LINE_SITE*2,self.pos[1] - self.size + init.LINE_SITE*2,
                        self.pos[2] + self.size - init.LINE_SITE*2,self.pos[3] + self.size - init.LINE_SITE*2,start = -np.rad2deg(angles[0]) ,extent=init.FIELD_OF_VIEW)
            self.showFieldOfView(lineAng,angles)

    def showDirectionVector(self,lineAng):
        init.CANVAS.delete(self.line)
        init.CANVAS.move(self.line,self.vx,self.vy)
        self.line = init.CANVAS.create_line(self.pos[2] - self.size + init.VECTOR_SIZE*np.cos(lineAng),self.pos[3] - self.size + init.VECTOR_SIZE*np.sin(lineAng),
                        self.pos[0] + self.size - init.VECTOR_SIZE*np.cos(lineAng),self.pos[1] + self.size - init.VECTOR_SIZE*np.sin(lineAng),arrow='first')

    def showFieldOfView(self,lineAng,angles):
        init.CANVAS.delete(self.fieldOfView)
        init.CANVAS.move(self.fieldOfView,self.vx,self.vy)
        self.fieldOfView = init.CANVAS.create_arc(self.pos[0] - self.size + init.LINE_SITE*2, self.pos[1] - self.size + init.LINE_SITE*2,
                        self.pos[2] + self.size - init.LINE_SITE*2,self.pos[3] + self.size - init.LINE_SITE*2, start = -np.rad2deg(angles[0]),extent=init.FIELD_OF_VIEW)

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
        st = angle - np.deg2rad(init.FIELD_OF_VIEW/2)
        if st < 0: 
            st = 2*np.pi + st
        vision = [st-np.pi/2,0] 
        return vision,angle

    def alertImpactWall(self):
        if self.pos[3]>= init.HEIGHT-init.LINE_SITE or self.pos[1]<=0+init.LINE_SITE: 
            self.alert = True
            #init.CANVAS.itemconfig(self.boid,fill='red')
        elif self.pos[2]>= init.WIDTH-init.LINE_SITE or self.pos[0]<=0+init.LINE_SITE:
            self.alert = True
            #init.CANVAS.itemconfig(self.boid,fill='red')
        else:
            self.alert = False
            #if init.SHOWGRIDCOLOR is False: 
                #init.CANVAS.itemconfig(self.boid,fill='blue')

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
            if (posM[0]-center[0])**2+(posM[1]-center[1])**2 <=init.LINE_SITE**2 or (posM[2]-center[0])**2+(posM[3]-center[1])**2 <=init.LINE_SITE**2:
                if vectAng(dirVec,vecFromSelf) < init.FIELD_OF_VIEW/2 : 
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
            if d <=init._COLLISION_DISTANCE_ and b != self and d !=0:
                centerM = boidUtil.centerPos(b)
                center = boidUtil.centerPos(self)
                diff = [centerM[0]-center[0],centerM[1]-center[1]]
                magnitudeDiff = magnitude(diff) - self.size/2 - b.size/2
                c[0] = c[0] - diff[0]/magnitudeDiff**2*init._COLLISION_FACTOR_
                c[1] = c[1] - diff[1]/magnitudeDiff**2*init._COLLISION_FACTOR_
        return c 

    def rule2(self,mates):
        """
        Implements rule 2 of the boid algo. This rule is in charged of handling the 
        alignment of nearby boids. This is done by finding the average vector of boids
        in the vicinity and then appyling that change to the boid in interest 
        """
        sum_x, sum_y = 0,0
        for m in mates:
            sum_x += m.vx
            sum_y += m.vy
        
        if len(mates)>0:
            average_x, average_y = sum_x/len(mates), sum_y/len(mates)
            return [(average_x-self.vx)*init._ALIGNMENT_FACTOR_, (average_y-self.vy)*init._ALIGNMENT_FACTOR_]
        else: 
            return [0,0]

    def rule3(self,mates):
        """
        Implements rule 3 of the boid algorithm. This is done by calculating the average position of the 
        group and making every boid attract to that position.
        """
        sum_x, sum_y = 0,0
        pos_center = boidUtil.centerPos(self)
        for m in mates:
            pos = boidUtil.centerPos(m)
            sum_x += pos[0]
            sum_y += pos[1]
         
        if len(mates)>0:
            average_x, average_y = sum_x/len(mates), sum_y/len(mates)
            return [(average_x-pos_center[0])*init._COHESION_FACTOR_,(average_y-pos_center[1])*init._COHESION_FACTOR_]
        else: 
            return [0,0]

s=space(ballInterest=ballInterest)

def run():
    init.CANVAS.pack()
    while (True):
        s.updateSpace()
        [b.move() for b in s.boids]
        init.TK.update()
        time.sleep(0.001)
    init.TK.mainloop()

if __name__ == '__main__':
    run()
    