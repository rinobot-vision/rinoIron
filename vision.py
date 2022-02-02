from robot import *

class vision:
    def __init__(self):
        self.robots = [robot(), robot(), robot()]
        self.enemy = [robot(), robot(), robot()]
        self.ball = dataState()
        self.centroidDef = self.centroidAtk = Point2f()

        #signals:
        #    void emit_info(vector<robot>,vector<robot>,Point2f,Point2f,dataState)

    def run(self):
        #emit emit_info(robots,enemy,centroidDef,centroidAtk,ball)
        pass

    def setCentroidDef(self, def_:Point2f):
        self.centroidDef = def_

    def setCentroidAtk(self, atk_:Point2f):
        self.centroidAtk = atk_

    def setRobots(self, robotsPoints:list):
        self.robots = robotsPoints
    
    def setBall(self, ball:dataState):
        self.ball = ball

    def getRobots(self):
        return self.robots

    def getEnemy(self):
        return self.enemy
    
    def getCentroidDef(self):
        return self.centroidDef
    
    def getCentroidAtk(self):
        return self.centroidAtk
    
    def getBall(self):
        return self.ball

