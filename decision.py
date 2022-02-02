from robot import *

FIXED3 = 0
FULL_ATK = 1
SAFE_FULL_ATK = 2
NS = 3
FUZZY_ = 4
KNN_ = 5
IMPOSTOR_ = 6
DOWNBELOW = 7
FUZZYT = 8
OFF = 9
CRUZAMENTO = 0 # 0 = desativado

#define SIT 4380

class Decision:
    def __init__(self):
        self.pathPoints = [Point2f(), Point2f(), Point2f()]
        self.clockChange = float()
        #self.fuzzy = Fuzzy()
        self.teamRobot = [robot(), robot(), robot()]
        self.tempT = float()
        self.centroidDef = self.centroidAtk = Point2f()
        self.ball = dataState()
        self.changePermission = bool()
        self.timeChange = float()
        self.firstTime = bool()
        self.path1 = self.path2 = int()
        self.strategy = int()
        self.swapRoles = True
        self.flagCrossing = False
        self.lastB = False
        #QTimer *timer1
        self.clockInvert = self.clockTroca = float()
        self.tempoTroca = float()
        self.clockStart = self.clockTrocaGS = self.clockTrocaFS = float()
        self.temp = self.timeTrocaGS = self.timeTrocaFS = float()
        self.flagTrocaGS = True
        self.flagTrocaFS = True
        self.LineD = 48.0
        self.G = self.D = self.S = self.F = self.L = self.M = int()
        self.lastG = self.lastD = self.lastS = self.lastM = self.lastL = self.lastF = self.lastW = int()
        self.lastCompanheiroFuzzy = 1
        self.strikerIndex = 1
        self.liberoIndex = 2
        #self.knnfunction = KNN()
        self.offLine = 115.0

        #fuzzy.start()
        self.changePermission = False
        self.firstTime = True
        
    def updateObjectives(self):
        robotFunc = [int(), int(), int()]
        if self.strategy == FIXED3:
            self.teamRobot[2].setFunction(GOALKEEPER)
            self.teamRobot[1].setFunction(DEFENDER)
            self.teamRobot[0].setFunction(STRIKER)
        elif self.strategy == FULL_ATK:
            pass
        elif self.strategy == NS:
            pass
        elif self.strategy == FUZZY_:
            pass
        elif self.strategy == KNN_:
            pass
        elif self.strategy == OFF:
            pass

    def getTeamRobots(self):
        return self.teamRobot

    def getCentroidDef(self):
        return self.centroidDef
    
    def getCentroidAtk(self):
        return self.centroidAtk

    def getBall(self):
        return self.ball

    def setRobots(self, r:list):
        self.teamRobot = r

    def setAreas(self, def_:Point2f, atk_:Point2f):
        self.centroidDef = def_
        self.centroidAtk = atk_
    
    def setBall(self, b:dataState):
        self.ball = b
    
    def setStrategy(self, s:int):
        self.strategy = s
    
    '''
    def setKnnInformation(self, k:KNN):
        self.knnfunction = k
    '''

    def getStrategy(self):
        return self.strategy
    