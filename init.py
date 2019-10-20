from tkinter import *


MAX_SPEED = 7
MIN_SPEED = 2
WIDTH = 1000
HEIGHT = 800
N_AREAS_WIDTH = 6
N_AREAS_HEIGHT = 6
TOTALGRID = N_AREAS_HEIGHT*N_AREAS_WIDTH

#Boid
MAX_SIZE = 25
MIN_SIZE = 15
VECTOR_SIZE = 30
LINE_SITE = 50
FIELD_OF_VIEW = 270
ANGLE_TOP = 40
ANGLE_BOTTOM = 70


#Axioms
_COLLISION_DISTANCE_ = LINE_SITE*2
_COLLISION_FACTOR_ = 7.5


NBALLS = 75
TOL = 10   #Make this as a function of total velocity 

COLORS = [ "pink", "cyan", "green", "yellow", "purple", "orange", "white", "black" ]
SHOWGRIDCOLOR = False
SHOWGRIDS = True

TK = Tk()
CANVAS = Canvas(TK,width=WIDTH,height=HEIGHT)