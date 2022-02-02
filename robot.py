from utils import *

GOALKEEPER = 0
DEFENDER = 1
STRIKER = 2
FAKE9 = 3
MIDFIELD = 4
WING = 5
VOLANTE = 6
FUZZYROBOT = 7
STRIKERAGAINSTEAM = 8
IMPOSTOR = 9
NEWSTRIKER = 12
LIBERO = 13
OFFDEFENDER = 14

class robot:
    def __init__(self):
        self.kinetic = dataState()
        self.function = int()
        self.kp = float()
        self.kd = float()
        self.ki = float()
        self.time = int()
        self.index = int()
        self.pathPoints = list()

    def setKp (self, k:float):
        self.kp = k

    def setKd (self, k:float):
        self.kd = k

    def setKi (self, k:float):
        self.ki = k

    def getKp(self):
        return self.kp

    def getKd(self):
        return self.kd

    def getKi(self):
        return self.ki

    def getDataState(self):
        return self.kinetic

    def getFunction(self):
        return self.function

    def setFunction(self, f:int):
        self.function = f

    def setPosition(self, pos:Point2f, ang:float, vel:Point2f):
        self.kinetic.pos = pos
        self.kinetic.vel = vel
        self.kinetic.angle = ang

    def setQuadrant(self, q:int):
        self.kinetic.quadrant = q

    def cleanPath(self):
        self.pathPoints.clear()

    def AddPathPoint(self, point:Point2f):
        self.pathPoints.append(point)

    def setTime(self, time:int):
        self.time = time

    def getTime(self):
        return self.time

    def getPosition(self):
        return self.kinetic.pos

    def getAngle(self):
        return self.kinetic.angle

    def getVelocidade(self):
        return self.kinetic.vel
