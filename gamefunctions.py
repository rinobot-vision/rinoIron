from robot import *
import time #variáveis TIME mudar de nome para TIME_
from navigation import *

class GameFunctions(Navigation):
    def __init__(self):
        self.airball = bool()
        self.fedUp = bool()
        self.atkPoin = Point2f()
        self.speedmf = float()
        self.teamRobot = [robot(), robot(), robot()]
        #objetoFuzzy = Fuzzy()
        #objetoFuzzy.start()
        self.littleChuteSituation = False
        self.freeballSituation = False
        self.penaltiSituation = False
        self.atkSituation = False
        self.atkSituationTiro = False
        self.flagAvoidGoalkepper = False
        self.flagCrossing = False
        self.flagAvoidDefender = False
        self.flagAvoidball = False
        self.flagStopOngoal = False
        self.flagMidWaiting = False
        self.atkSituationInv = False
        self.plotState = False
        self.kickPState = False
        self.flagGoAhead = False
        self.flagKickball = False
        self.penaultyPermission = False
        self.flagGrab = False
        self.flagGrabM = False

        self.defenderLine = 38
        self.volanteLine = 38
        self.lastX = self.defenderLine
        self.offLine = 115

        self.strategy = int()
        self.indexRobot = int()
        self.ball = dataState()
        self.StrikeRepulsiveM = Point2f()
        self.StrikeRepulsive = Point2f()

        self.centroidDef = self.centroidAtk = Point2f()

        self.tempoRepulsive = 0
        self.tempoStopRepulsive = 0
        self.tempoRepulsiveM = 0
        self.tempoStopRepulsiveM = 0

        self.ClockStartRM = float()
        self.ClockStopRM = float()
        self.ClockStartR = float()
        self.ClockStopR = float()

    def run(self):
        if len(self.teamRobot) > self.indexRobot:
            if self.teamRobot[self.indexRobot].getFunction() == GOALKEEPER:
                self.goalkeeper()
            elif self.teamRobot[self.indexRobot].getFunction() == DEFENDER:
                self.defender()
            elif self.teamRobot[self.indexRobot].getFunction() == STRIKER:
                self.striker()
            elif self.teamRobot[self.indexRobot].getFunction() == FAKE9:
                self.fake9()
            elif self.teamRobot[self.indexRobot].getFunction() == MIDFIELD:
                self.midfield()
            elif self.teamRobot[self.indexRobot].getFunction() == WING:
                self.wing()
            elif self.teamRobot[self.indexRobot].getFunction() == FUZZYROBOT:
                self.fuzzy()
            elif self.teamRobot[self.indexRobot].getFunction() == IMPOSTOR:
                self.impostor()
            elif self.teamRobot[self.indexRobot].getFunction() == LIBERO:
                self.libero()
            elif self.teamRobot[self.indexRobot].getFunction() == OFFDEFENDER:
                self.offdefender()
            if self.getPlot() == True:
                self.PlotPath(1,self.teamRobot[self.indexRobot])

    def getgSizeW(self):
        return self.gSizeW

    def setgSizeW(self, a:float):
        self.gSizeW = a

    def getdeW(self):
        return self.deW

    def setdeW(self, a:float):
        self.deW = a

    def getkrW(self):
        return self.KrW

    def setkrW(self, a:float):
        self.KrW = a

    def getkLarg(self):
        return self.k_larg

    def setkLarg(self, a:float):
        self.k_larg = a

    def setRobots(self, robots:list):
        self.teamRobot = robots

    def setAreas(self, defs:Point2f, atk:Point2f):
        self.centroidDef = defs
        self.centroidAtk = atk

    def setBall(self, b:dataState):
        self.ball = b

    def getGoal(self):
        return self.goal

    def getDirection(self):
        return self.thePhi

    def getDefenderLine(self):
        return self.defenderLine

    def getVolanteLine(self):
        return self.volanteLine

    def setDefenderLine(self, possld:float):
        self.defenderLine = possld

    def setAtkSituation(self, a:bool):
        self.atkSituation = a

    def setAtkSituationTiro(self, a:bool):
        self.atkSituationTiro = a

    def setAtkSituationInv(self, a:bool):
        self.atkSituationInv = a

    def getAtkSituation(self):
        return self.atkSituation

    def getAtkSituationTiro(self):
        return self.atkSituationTiro

    def setCrossing(self, a:bool):
        self.flagCrossing = a

    def getCrossing(self):
        return self.flagCrossing

    def getAtkSituationInv(self):
        return self.atkSituationInv

    def setStopOnGoal(self, a:bool):
        self.flagStopOnGoal = a

    def getStopOnGoal(self):
        return self.flagStopOnGoal
 
    def setPlot(self, state:bool):
        self.plotState = state

    def getPlot(self):
        return self.plotState

    def setKickState(self, state:bool):
        self.kickPState = state

    def getKickState(self):
        return self.kickPState

    def setPenalty(self, state: bool):
        self.penaultyPermission = state

    def getPenalty(self):
        return self.penaultyPermission

    def setStrategy(self, s:int):
        self.strategy = s

    def getStrategy(self):
        return self.strategy
    
    def kickBall(self):
        self.goal = self.ball.pos
        self.thePhi = angleTwoPoints(self.teamRobot[self.indexRobot].getDataState().pos,self.ball.pos)

    def setFreeBall(self, state:bool):
        self.freeBallSituation = state

    def getFreeball(self):
        return self.freeBallSituation

    def setlittleChute(self,state:bool):
        self.littleChuteSituation = state

    def getlittleChute(self):
        return self.littleChuteSituation

    def setIndex(self, index:int):
        self.indexRobot = index

    def striker(self):
        self.flagGoAhead = False
        self.setStopOnGoal(False)
        self.airball = False
        self.flagAvoidDefender = False
        self.de = self.deW
        self.Kr = self.KrW
        robot = self.teamRobot[self.indexRobot].getDataState()

        for Robot in self.teamRobot:
            if Robot.getFunction() == GOALKEEPER:
                goalk = Robot.getDataState()
                break      

        if self.ball.vel.x>0 and euclidean_dist(robot.pos, self.ball.pos)>12:
            time_ = 0.07
        else:
            time_ = 0
        
        self.ballprev = Point2f(self.ball.pos.x + time_*self.ball.vel.x, self.ball.pos.y + time_*self.ball.vel.y)
        self.goal = self.ballprev
        
        if robot.pos.x > self.goal.x:
            self.gSize = 0
        else:
            self.gSize = 0.6

        if self.ball.pos.x < self.centroidDef.x + self.defenderLine and self.ball.pos.y < self.centroidDef.y + self.defenderLine and self.ball.pos.y > self.centroidDef.y - self.defenderLine:
            self.goal.x = self.centroidDef.x + 24
            self.goal.y = self.ball.pos.y
            self.setStopOnGoal(True)
            self.airball = True

        elif self.ball.pos.x < self.centroidDef.x + self.defenderLine:
            if self.ball.pos.y < self.centroidDef.y - 30:
                self.goal.x = self.defenderLine - 3
                self.goal.y = self.centroidDef.y - 25
                self.setStopOnGoal(True)

            if self.ball.pos.y > self.centroidDef.y + 30:
                self.goal.x = self.defenderLine + 3
                self.goal.y = self.centroidDef.y + 25
                self.setStopOnGoal(True)

        else:
            if self.ball.pos.y < 15  or self.ball.pos.y > 115:
                if self.ball.pos.y < self.centroidAtk.y:
                    self.thetaDir = (-25*pi/180) 

                if self.ball.pos.y > self.centroidAtk.y:
                    self.thetaDir = (25*pi/180)

            else:
                aux = Point2f(self.centroidAtk.x, robot.pos.y)
                theta = angleTwoPoints(self.ball.pos,robot.pos)
                alpha = angleTwoPoints(robot.pos,self.ball.pos)
                cat_ad = euclidean_dist(robot.pos,aux)
                cat_op = cat_ad*tan(theta*pi/180)
                minLimit = Point2f(self.centroidAtk.x, 55)
                maxLimit = Point2f(self.centroidAtk.x, 75)

                if self.ball.pos.y < robot.pos.y:
                    ang_point = Point2f(self.centroidAtk.x, robot.pos.y - abs(cat_op))

                else:
                    ang_point = Point2f(self.centroidAtk.x, robot.pos.y + abs(cat_op))

                if (ang_point.y < 80) and (ang_point.y > 50) and robot.pos.x < self.ball.pos.x:
                    self.thetaDir = (alpha*pi/180)
                    self.atkPoint = ang_point
                    self.setAtkSituation(True)
                    self.flagGoAhead = True
                    #self.thetaDir = 0

                else:
                    if robot.pos.x < self.ball.pos.x:
                        if ang_point.y > 80:
                            self.thetaDir = angleTwoPoints(self.ball.pos,maxLimit)*pi/180
                            self.atkPoint = maxLimit

                        elif ang_point.y < 50:
                            self.thetaDir = angleTwoPoints(self.ball.pos,minLimit)*pi/180
                            self.atkPoint = minLimit

                        else:
                            self.thetaDir = 0
                            self.atkPoint = self.centroidAtk

                    else:
                        self.thetaDir = 0
                    self.setAtkSituation(False)
        
                if self.ball.pos.y >= 50 and self.ball.pos.y <= 80 and (robot.pos.y < 50 or robot.pos.y > 80):
                    self.thetaDir = 0
        
                if self.ball.pos.x > self.centroidAtk.x - 10:
                
                    if self.ball.pos.y > self.centroidDef.y - 65 and self.ball.pos.y < self.centroidDef.y - 20:
                        self.thetaDir = (40*pi/180)
                
                    if self.ball.pos.y < self.centroidDef.y + 65 and self.ball.pos.y > self.centroidDef.y + 20:
                        self.thetaDir = (-40*pi/180)
                
            if self.ball.pos.y < 20  or self.ball.pos.y > 110:
                if robot.pos.x > self.ball.pos.x:
                    if self.ball.pos.y < 35  and robot.pos.y < self.ball.pos.y:
                        self.thetaDir = (-90 * pi / 180)
                        self.de = 0.7
                        self.Kr = 0
                    if self.ball.pos.y > 95 and robot.pos.y > self.ball.pos.y:
                        self.thetaDir = (90 * pi / 180)
                        self.de = 0.7
                        self.Kr = 0

        if self.goal.x > self.centroidDef.x + 35:
            if self.goal.x > self.centroidAtk.x-10:
                if robot.pos.x > self.goal.x:
                    self.de = 0.7
                    self.Kr = 0
                    self.thetaDir = 0
            else:
                if robot.pos.y<25 or robot.pos.y>105:
                    self.de = 1
                    self.Kr = 0
    
        else:
            self.de = 1
            self.Kr = 0
            self.gSize = self.gSizeW

            if self.goal.y > 65:
                self.thetaDir = 90*pi/180
            else:
                self.thetaDir = -90*pi/180

            if self.ball.pos.x < 20:
                if self.goal.y > 65 and robot.pos.y < self.ball.pos.y:
                    self.thetaDir = angleTwoPoints(robot.pos,self.ball.pos)*pi/180
                if self.goal.y < 65 and robot.pos.y > self.ball.pos.y:
                    self.thetaDir = angleTwoPoints(robot.pos,self.ball.pos)*pi/180
            
        
        for Robot in self.teamRobot:
            if Robot.getFunction() == DEFENDER:
                defender = Robot.getDataState().pos
                break
        
    
        if self.ball.pos.x < self.centroidDef.x + self.defenderLine:
            if self.ball.pos.y < self.centroidDef.y - 35: 
                if robot.pos.x < self.centroidDef.x + self.defenderLine:
                    if robot.pos.y > goalk.pos.y - 10:
                        flagAvoidGoalkepper = True
                    else:
                        flagAvoidGoalkepper = False
                else:
                    flagAvoidGoalkepper = False

            elif self.ball.pos.y > self.centroidDef.y + 35 :
                if robot.pos.x < self.centroidDef.x + self.defenderLine:
                    if robot.pos.y < goalk.pos.y + 10:
                        flagAvoidGoalkepper = True
                    else:
                        flagAvoidGoalkepper = False
                else:
                    flagAvoidGoalkepper = False
        
        if flagAvoidGoalkepper:
            self.k_larg = 0.08
            self.thePhi = self.repulsiveMath(self.teamRobot[self.indexRobot].getDataState(), goalk.pos) 
        else:
            if euclidean_dist(robot.pos,defender) < 15 and euclidean_dist(self.ball.pos,defender) > 15:
                self.flagAvoidDefender = True
                self.k_larg = 0.06
                self.thePhi = self.repulsiveMath(self.teamRobot[self.indexRobot].getDataState(), defender)
            else:
                #if(flagGoAhead)
                    #thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal)
                
                if self.flagStopOnGoal:
                    self.thePhi = angleTwoPoints(self.teamRobot[self.indexRobot].getDataState().pos, self.goal)
                else:
                    self.univectorField(self.teamRobot[self.indexRobot].getDataState(), defender)
                
    
        if self.penaultyPermission == True or self.freeballSituation == True:
            self.kickball()
            self.setAtkSituation(True)
    
    
        if self.tempoRepulsive == 0:
            self.ClockStartR = time.time()
            self.tempoStopRepulsive = 0
            self.flagGrab = False
            self.StrikeRepulsive = robot.pos
        
        self.tempoRepulsive = (time.time() - self.ClockStartR)
        
        if self.tempoRepulsive >= 3:

            if euclidean_dist(self.StrikeRepulsive, robot.pos) <= 5 or self.flagGrab == True:
                self.k_larg = 0.01
                self.thePhi = ajustaAngulo(self.thePhi + 90)
                if self.flagGrab == False:
                    self.ClockStopR = time.time()
                self.flagGrab = True
            else:
                self.tempoRepulsive = 0
        
        if self.flagGrab == True:
            self.tempoStopRepulsive = time.time() - self.ClockStopR

        if self.tempoStopRepulsive >= 0.5:
            self.tempoRepulsive = 0
            self.tempoStopRepulsive = 0
            self.flagGrab = False
    
    def defender(self):
        self.flagAvoidGoalkepper = False
        self.flagKickball = False
        robot = self.teamRobot[self.indexRobot].self.getDataState()
        
        for Robot in self.teamRobot:
            if Robot.self.getFunction() == GOALKEEPER:
                goalk = Robot.self.getDataState()
                break
        
        if self.ball.pos.x > self.centroidDef.x + self.defenderLine:
            self.goal.x = self.centroidDef.x + self.defenderLine
            self.goal.y = self.ball.pos.y

            if robot.pos.x < self.centroidDef.x + self.defenderLine - 15:
                if self.strategy == NS:
                    self.flagAvoidGoalkepper = False
                else:
                    self.flagAvoidGoalkepper = True
            
            else:
                self.flagAvoidGoalkepper = False
        else:
            if self.ball.pos.y < 30:
                self.goal = self.ball.pos
                self.flagKickball = True
                self.gSize = self.gSizeW
                self.de = self.deW
                self.Kr = self.KrW
                self.thetaDir = 55.0
            
            elif self.ball.pos.y > 100:
                self.goal = self.ball.pos
                self.flagKickself.ball = True
                self.gSize = self.gSizeW
                self.de = self.deW
                self.Kr = self.KrW
                self.thetaDir = -55.0

            else:
                self.goal.x = self.defenderLine + self.centroidDef.x
                self.goal.y = self.ball.pos.y

                if self.ball.pos.x < self.centroidDef.x + 15:
                    if self.ball.pos.y > self.centroidDef.y:
                        self.goal.x = self.centroidDef.x + 37
                        self.goal.y = self.centroidDef.y + 42
                    
                    else:
                        self.goal.x = self.centroidDef.x + 37
                        self.goal.y = self.centroidDef.y - 42

        if self.flagAvoidGoalkepper == True:
            self.k_larg = 0.1
            self.thePhi = self.repulsiveMath(self.teamRobot[self.indexRobot].getDataState(), goalk.pos)
        
        elif self.flagAvoidGoalkepper == False:
            if(self.flagAvoidself.ball == True):
                self.k_larg = 0.1
                self.thePhi = self.repulsiveMath(self.teamRobot[self.indexRobot].getDataState(), self.ball.pos)
            
            else:
                if(self.flagKickself.ball):
                    self.univectorField(robot, Point2f(0,0))
                
                else:
                    self.thePhi = angleTwoPoints(self.teamRobot[self.indexRobot].getDataState().pos,self.goal)

    def goalkeeper(self):
        distXballgkp = self.ball.pos.x - (self.centroidDef.x +11)
        if fabs(self.ball.vel.x ) < 30:
            previsionTime = 0
        
        else:
            previsionTime = (distXballgkp / self.ball.vel.x)
            
        previsionYball = (self.ball.vel.y * previsionTime) + self.ball.pos.y

        robot = self.teamRobot[self.indexRobot].getDataState()
        for Robot in self.teamRobot:
            if Robot.getFunction() == DEFENDER:
                defenderPos = Robot.getDataState().pos
                break

        if self.ball.pos.x < self.centroidDef.x + 90:
            if previsionYball < self.centroidDef.y - 20:
                self.goal.x = self.centroidDef.x + 7
                self.goal.y = self.centroidDef.y - 15
            
            elif previsionYball> self.centroidDef.y + 20:
                self.goal.x = self.centroidDef.x + 7
                self.goal.y = self.centroidDef.y + 15
            
            else:
                self.goal.x = self.centroidDef.x + 7
                self.goal.y = self.ball.pos.y

        else:
            self.goal.x = self.centroidDef.x + 7
            self.goal.y = self.centroidDef.y

        if self.ball.pos.x < robot.pos.x - 4:
            if self.ball.pos.x < self.centroidDef.x+15:
                self.k_larg = 0.045
            
            else:
                self.k_larg = 0.04
            
            self.thePhi = self.repulsiveMath(robot, self.ball.pos)
        
        elif defenderPos.x < robot.pos.x:
            if self.ball.pos.x < self.centroidDef.x+15:
                self.k_larg = 0.045
            
            else:
                self.k_larg = 0.04
            
            self.thePhi = self.repulsiveMath(robot, defenderPos)
        
        else:
            self.thePhi = angleTwoPoints(robot.pos,self.goal)

    def fake9(self):
        pass
    def midfield(self):
        pass
    def wing(self):
        pass
    def fuzzy(self):
        pass
    def impostor(self):
        pass
    def libero(self):
        pass
    def offdefender(self):
        pass

    def PlotPath(self, i:int, robot:robot):
        self.plotThePhi.clear()
        if(i == 0):
            self.atkPaths.clear()
        virtualRobot = dataState(pos = robot.getDataState().pos)
        r = 5
        cont = 0
        
        for Robot in self.teamRobot:
            if Robot.getFunction() == DEFENDER:
                enemy = Robot.getDataState().pos
                break 

        for Robot in self.teamRobot:
            if Robot.getFunction() == STRIKER:
                killer = Robot.getDataState()
                break 

        for Robot in self.teamRobot:
            if Robot.getFunction() == DEFENDER:
                defender = Robot.getDataState().pos
                break 

        for Robot in self.teamRobot:
            if Robot.getFunction() == GOALKEEPER:
                goalk = Robot.getDataState()
                break 

        while (euclidean_dist(virtualRobot.pos,self.goal) > 3) and (cont < 215): #(fabs(virtualRobot.pos.x - goal.x) > 1) and (fabs(virtualRobot.pos.y - goal.y) > 1))
            self.AddPlotPoint(virtualRobot.pos)
            

            if robot.getFunction()==GOALKEEPER:
                ang = angleTwoPoints(virtualRobot.pos,self.goal)*pi/180
            elif robot.getFunction()==WING:
                ang = (ajustaAngulo(self.univectorFieldForPlot(virtualRobot,enemy)))*pi/180
            elif robot.getFunction()==DEFENDER:
                if self.flagAvoidGoalkepper:
                    ang = self.repulsiveMath(virtualRobot, goalk.pos)*pi/180
                else:
                    if self.flagAvoidBall:
                        ang = self.repulsiveMath(virtualRobot, self.ball.pos)*pi/180
                    else:
                        ang = angleTwoPoints(virtualRobot.pos,self.goal)*pi/180
            elif robot.getFunction()==STRIKER:
                if self.flagAvoidGoalkepper:
                    ang = self.repulsiveMath(virtualRobot, goalk.pos)*pi/180
                else:
                    if self.flagAvoidDefender:
                        ang = self.repulsiveMath(virtualRobot, defender)*pi/180
                    else:
                        ang = (ajustaAngulo(self.univectorFieldForPlot(virtualRobot,enemy)))*pi/180
            elif robot.getFunction()==FAKE9:
                if self.flagAvoidGoalkepper:
                    for Robot in self.teamRobot:
                        if Robot.getFunction() == GOALKEEPER:
                            goalk = Robot.getDataState()
                            break 

                    ang = self.repulsiveMath(virtualRobot, goalk.pos)*pi/180
                else:
                    if self.flagAvoidDefender:
                        for Robot in self.teamRobot:
                            if Robot.getFunction() == DEFENDER:
                                goalk = Robot.getDataState().pos
                                break 

                        ang = self.repulsiveMath(virtualRobot, defender)*pi/180
                    else:
                        if virtualRobot.pos.x < killer.pos.x - 10:
                            ang = angleTwoPoints(virtualRobot.pos, self.goal)*pi/180
                        else:
                            ang = self.repulsiveMath(virtualRobot, self.ball.pos)*pi/180
            else:
                ang = angleTwoPoints(virtualRobot.pos,self.goal)*pi/180

            virtualRobot.pos.x = virtualRobot.pos.x + r*cos(ang)
            virtualRobot.pos.y = virtualRobot.pos.y + r*sin(ang)

            cont += 1
