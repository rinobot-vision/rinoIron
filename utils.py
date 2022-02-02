from math import *
import numpy as np

STRATEGY = 0
NUMROBOTS = 3
FPS = 100
#define CLEAR(x) memset(&(x), 0, sizeof(x))

class Point2f:
    def __init__(self, x:float ,y:float):
        self.x = x
        self.y = y
    
    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        return Point2f(x,y)

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        return Point2f(x,y)

class dataState:
    def __init__(self, pos:Point2f=None, vel:Point2f=None, angle:float=None, quadrant:int=None):
        self.pos = pos
        self.vel = vel
        self.angle = angle
        self.quadrant = quadrant

def euclidean_dist(p:Point2f, q:Point2f):
    return sqrt((q.x - p.x)**2 + (q.y - p.y)**2)

def angleTwoPoints(p:Point2f, q:Point2f):
    vec = q - p
    angle = atan2(vec.y,vec.x)*180/pi
    return angle

def ajustaAngulo(angle:float):
    while (angle < -180):
        angle = angle + 360
    while (angle > 180):
        angle = angle - 360
    return angle

def leastSquares2f(pos:list):
    t = []
    for i in range(len(pos)):
        t.append((len(pos) - i - 1)/FPS)

    M = np.zeros((len(pos), 3))
    Vx = np.zeros((len(pos), 1))
    Vy = np.zeros((len(pos), 1))

    for i in range(len(pos)):
        for j in range(3):
            if j == 0:
                M[i,j] = 1
            
            elif j == 1:
                M[i,j] = t[i]
            
            else:
                M[i,j] = t[i]*t[i]
            
        Vx[i,0] = pos[i].x
        Vy[i,0] = pos[i].y
    
    Ux = (M.T*M)**(-1)*(M.T*Vx)
    Uy = (M.T*M)**(-1)*(M.T*Vy)
    T = np.matrix([[1],[t[0]],[t[0]**2]])
    T_diff = np.matrix([[0],[1],[t[0]*2]])

    result = []
    aux = Point2f()
    aux.x = (Ux.T()*T)[0,0]
    aux.y = (Uy.T()*T)[0,0]
    result.append(aux)
    aux.x = (Ux.T()*T_diff)[0,0]
    aux.y = (Uy.T()*T_diff)[0,0]
    result.append(aux)

    return result

def leastSquares(angle:list):
    t = []
    for i in range(len(angle)):
        t.append((len(angle) - i - 1)/FPS)

    M = np.zeros((len(angle), 3))
    V = np.zeros((len(angle), 1))

    for i in range(len(angle)):
        for j in range(3):
            if j == 0:
                M[i,j] = 1
            
            elif j == 1:
                M[i,j] = t[i]
            
            else:
                M[i,j] = t[i]*t[i]
            
        V[i,0] = angle[i]
    
    U = (M.T*M)**(-1)*(M.T*V)
    T = np.matrix([[1],[t[0]],[t[0]**2]])
    T_diff = np.matrix([[0],[1],[t[0]*2]])

    result = []
    result.append(U.T*T[0,0])
    result.append(U.T*T_diff[0,0])

    return result