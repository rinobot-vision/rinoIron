from robot import *
from decision import *
from gamefunctions import *
from mover import *

class gamewindow:
    def __init__(self):
        self.posBall = list() #point2f
        self.posRobot0 = list() #point2f
        self.posRobot1 = list() #point2f
        self.posRobot2 = list() #point2f
        self.deecision = Decision()
        self.robotFunctions = [GameFunctions(),GameFunctions(),GameFunctions()]
        self.mover = [Mover(),Mover(),Mover()]
        for i in range(NUMROBOTS):
            self.mover[i].setIndex(i)
            self.robotFunctions[i].setIndex(i)

    def updateInfo(self, robots:list, enemy:list, def_:Point2f, atk_:Point2f, ball:dataState):
        self.deecision.setRobots(robots)
        self.deecision.setAreas(def_,atk_)
        self.deecision.setBall(ball)
        self.deecision.setStrategy(STRATEGY)
        self.deecision.updateObjectives()
        teste = self.deecision.getTeamRobots()
        self.updateRunFunctions(teste,self.deecision.getCentroidDef(),self.deecision.getCentroidAtk(),self.deecision.getBall())


    def updateRunFunctions(self, robots:list, def_:Point2f , atk_:Point2f , b:dataState):
        for i in range(NUMROBOTS):
            self.robotFunctions[i].setRobots(robots)
            self.robotFunctions[i].setAreas(def_,atk_)
            self.robotFunctions[i].setBall(b)
            self.robotFunctions[i].setStrategy(STRATEGY)
            self.mover[i].setRobots(robots)
            self.mover[i].setAreas(def_,atk_)
            self.mover[i].setBall(b)
    
        #FOR_EACH_ROBOT_FUNCTIONS(start())
        #FOR_EACH_ROBOT_FUNCTIONS(wait())

        self.mover[0].setGameFunctions(self.robotFunctions[0],self.robotFunctions[1],self.robotFunctions[2])
        self.mover[1].setGameFunctions(self.robotFunctions[0],self.robotFunctions[1],self.robotFunctions[2])
        self.mover[2].setGameFunctions(self.robotFunctions[0],self.robotFunctions[1],self.robotFunctions[2])

        #FOR_EACH_MOVER(start())
        #FOR_EACH_MOVER(wait())



    def EnviaVelocidades(self, Velocidades:list):

        for i in range(3):
            Velocidades[i].x = self.mover[i].getLVel()
            Velocidades[i].y = self.mover[i].getRVel()
        

