from gamefunctions import *

#QMutex mutex
#só era usado no destrutor, então tiramos

class Mover:
    def __init__(self):
        self.vMax = 70.0
        self.kp = 19.0
        self.kd = 2.5
        self.temp = 0.0
        self.tempoTroca = 0.0

        self.posTemp = Point2f()
        self.prevGoal = Point2f()

        self.inverte = False
        self.sentido = False

        self.clockInvert  = self.clockTroca  = self.clockAceleration = ()
        self.clockStart  = ()

        self.robotFunctions = [GameFunctions(), GameFunctions(), GameFunctions()]
        self.teamRobot = [robot(), robot(), robot()] #robot
        self.lastRobot = Point2f()
        self.centroidDef  = self.centroidAtk = Point2f()
        self.ball = dataState()
        self.robotGains = np.matrix([])

        self.l = 0.04
        self.alpha = ()
        self.lastAlpha = ()
        self.lastVel = ()
        self.alphaS = ()
        self.offLine = 115.0
        self.distGiro = 9.0
        self.distGiroGoleiro = 8.0
        self.velGiroLado = 0.8
        self.velGiroAtk = 0.5
        self.velGiroGol = 1.0
        self.velGiroPenalty = 2.0
        self.lVel = ()
        self.rVel = ()

        self.indexRobot = int()
        self.robotObstCont = 0
        self.robotTimeCont = 0
        self.strategy = int()

        self.robotDirection = True
        self.firstAceleration = True
        self.airball = bool()

    def run(self):
        if len(self.teamRobot) > self.indexRobot:
            #updateGains()
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
            elif self.teamRobot[self.indexRobot].getFunction() == VOLANTE:
                self.volante()
            elif self.teamRobot[self.indexRobot].getFunction() == FUZZYROBOT:
                self.fuzzy() 
            elif self.teamRobot[self.indexRobot].getFunction() == IMPOSTOR:
                self.impostor()
            elif self.teamRobot[self.indexRobot].getFunction() == LIBERO:
                self.libero()
            elif self.teamRobot[self.indexRobot].getFunction() == NEWSTRIKER:
                self.newstriker()
            elif self.teamRobot[self.indexRobot].getFunction() == OFFDEFENDER:
                self.offdefender()

    def setVMax(self, v):
        self.vMax = v

    def setIndex(self,index:int):
        self.indexRobot = index

    def setRobots(self, r:list):
        self.teamRobot = r

    def setAreas(self, def_:Point2f, atk_:Point2f):
        self.centroidDef = def_
        self.centroidAtk = atk_

    def setBall(self, b:dataState):
        self.ball = b

    def getVMax(self):
        return self.vMax

    def getLVel(self):
        return self.lVel

    def getRVel(self):
        return self.rVel

    def setGameFunctions(self, g0:GameFunctions, g1:GameFunctions, g2:GameFunctions):
        self.robotFunctions[0] = g0
        self.robotFunctions[1] = g1
        self.robotFunctions[2] = g2

    def goalkeeper(self):
        vFollow = 0.6
        vMaxPrevision = 1.2
        limiarTheta = 90.0
        deltaLimiar = 30.0
        vMaxGol = 0.7

        vDeltaGol = vMaxGol

        robotPos = self.teamRobot[self.indexRobot].getDataState().pos
        robotAngle = self.teamRobot[self.indexRobot].getDataState().angle
        if self.ball.vel.x < -30:
            time_ = (robotPos.x+5 - self.ball.pos.x)/self.ball.vel.x
        
        else:
            time_ = 0
        

        positionY = self.ball.pos.y + time_*self.ball.vel.y

        if positionY < 45:
            positionY = 45
        
        elif positionY > 85:
            positionY = 85
        

        self.prevGoal = robotPos
        if self.ball.vel.x < -10 and (positionY > self.centroidDef.y-20) and (positionY < self.centroidDef.y+20):
            vPrev = fabs(0.01 * (positionY - robotPos.y) / time_)

            if vPrev > vMaxPrevision:
                vPrev = vMaxPrevision
            
            elif vPrev < 0.5:
                vPrev = 0.5
            
            #vPrev = vMax
            vDeltaPrev = vPrev * 1

            if robotPos.x < self.centroidDef.x + 7 and robotPos.x > self.centroidDef.x - 5:
                self.prevGoal = Point2f(robotPos.x, positionY)
            
            else:
                self.prevGoal = Point2f(self.centroidDef.x + 7, positionY)
            
            theta = angleTwoPoints(robotPos, self.prevGoal)
            self.alpha = theta - robotAngle
            self.alpha = ajustaAngulo(self.alpha)
            if fabs(self.alpha) <= limiarTheta:
                v = -vDeltaPrev * fabs(self.alpha) / limiarTheta + vPrev
                w = self.kp * self.alpha / 180 + self.kd * (self.alpha - self.lastAlpha)
                limiarTheta = 90 - deltaLimiar
            
            else:          
                self.alpha = ajustaAngulo(self.alpha + 180)
                v = vDeltaPrev * fabs(self.alpha) / limiarTheta - vPrev
                w = self.kp * self.alpha / 180 + self.kd * (self.alpha - self.lastalpha)
                limiarTheta = 90 + deltaLimiar
            

            if (fabs(robotPos.x - self.prevGoal.x) < 5) and (fabs(robotPos.y - self.prevGoal.y) < 4):
                v = 0
                if fabs(robotAngle) > 80 and fabs(robotAngle) < 100:
                    w = 0
                
                else:
                    self.alpha = 90 - robotAngle
                    self.alpha = ajustaAngulo(self.alpha)
                    if fabs(self.alpha) <= limiarTheta:
                        w = self.kp*self.alpha/180 + self.kd*(self.alpha - self.lastAlpha)
                    
                    else:
                        self.alpha = ajustaAngulo(self.alpha + 180)
                        w = self.kp*self.alpha/180 + self.kd*(self.alpha - self.lastAlpha)
        else:
            if (self.ball.pos.x < self.centroidDef.x + 70) and (self.ball.pos.y > self.centroidDef.y - 25) and (self.ball.pos.y < self.centroidDef.y + 25) and (robotPos.x < self.centroidDef.x+5) and (robotPos.x >= self.centroidDef.x -5):
                theta = self.robotFunctions[self.indexRobot].getDirection()
                self.alpha = theta - robotAngle
                self.alpha = ajustaAngulo(self.alpha)
                if fabs(self.alpha) <= limiarTheta:
                    w = self.kp*self.alpha/180 + self.kd*(self.alpha - self.lastAlpha)
                    limiarTheta = 90 - deltaLimiar
                
                else:
                    self.alpha = ajustaAngulo(self.alpha+180)
                    w = self.kp*self.alpha/180 + self.kd*(self.alpha - self.lastAlpha)
                    limiarTheta = 90 + deltaLimiar
                

                if fabs(robotPos.y-self.ball.pos.y) < 3:
                    v = 0
                
                elif (robotAngle > 0) and (self.ball.pos.y < robotPos.y):
                    v = -vFollow
                
                elif (robotAngle > 0) and (self.ball.pos.y > robotPos.y):
                    v = vFollow
                
                elif (robotAngle < 0) and (self.ball.pos.y < robotPos.y):
                    v = vFollow
                
                elif (robotAngle < 0) and (self.ball.pos.y > robotPos.y):
                    v = -vFollow
                
            
            else:
                theta = self.robotFunctions[self.indexRobot].getDirection()
                self.alpha = theta - robotAngle
                self.alpha = ajustaAngulo(self.alpha)

                if fabs(self.alpha) <= limiarTheta:
                    v = -vDeltaGol*fabs(self.alpha)/limiarTheta + vMaxGol
                    w = self.kp*self.alpha/180 + self.kd*(self.alpha - self.lastAlpha)
                    limiarTheta = 90 - deltaLimiar
                
                else:
                    self.alpha = ajustaAngulo(self.alpha+180)
                    v = vDeltaGol*fabs(self.alpha)/limiarTheta - vMaxGol
                    w = self.kp*self.alpha/180 + self.kd*(self.alpha - self.lastAlpha)
                    limiarTheta = 90 + deltaLimiar
                

                if (fabs(robotPos.x - self.robotFunctions[self.indexRobot] .getGoal().x <2.5)) and (((self.robotFunctions[self.indexRobot].getGoal().y - robotPos.y<1) and (self.ball.pos.y>65)) or ((robotPos.y - self.robotFunctions[self.indexRobot].getGoal().y <1) and (self.ball.pos.y<65))):
                    v = 0
                    if(fabs(robotAngle) > 85 and fabs(robotAngle) < 95):
                        w = 0
                    
                    else:
                        self.alpha = 90 - robotAngle
                        self.alpha = ajustaAngulo(self.alpha)
                        if fabs(self.alpha) <= limiarTheta:
                            w = self.kp*self.alpha/180
                        
                        else:
                            self.alpha = ajustaAngulo(self.alpha+180)
                            w = self.kp*self.alpha/180
    
        if w > 30:
            w = 30
        if w < -30:
            w = -30

        self.lVel = (v - w*self.l)*100
        self.rVel = (v + w*self.l)*100

        self.lastAlpha = self.alpha

        self.rotate_def()

    def defender(self):
        vMaxD = 0.65
        vFollow = 0.50
        vMaxPrevision = 0.85
        vDelta = 1*vMaxD
        limiarTheta = 90.0
        deltaLimiar = 30.0
        distGiroGol = 10.0
        velGiroGol = 1.0

        self.airball = False

        if self.ball.pos.x < self.centroidDef.x +15 and self.ball.pos.y < self.centroidDef.y + 40 and self.ball.pos.y > self.centroidDef.y - 40:
            self.airball = True
        

        robotPos = Point2f(self.teamRobot[self.indexRobot].getDataState().pos)
        robotAngle = self.self.teamRobot[self.indexRobot].getDataState().angle

        if self.tempoTroca == 0:
            if self.temp == 0:
                self.posTemp = robotPos
                self.clockInvert = time.time()
            

            if euclidean_dist(robotPos, self.posTemp) <= 0.2:
                self.temp = time.time() - self.clockInvert

                if self.temp >= 3:
                    self.inverte = True  #?????
                    self.inverte = False #?????
                    self.clockTroca = time.time()
                    self.temp = 0
                else:
                    self.inverte = False
                
            else:
                self.inverte = False
                self.temp = 0
            
        
        self.inverte = False
        if self.inverte == True:
            self.tempoTroca = time.time() - self.clockInvert
            if self.tempoTroca > 3:
                self.tempoTroca = 0
        

        time_ = (robotPos.x+5 - self.ball.pos.x)/self.ball.vel.x
        positionY = self.ball.pos.y + time_*self.ball.vel.y

        if positionY < 0:
            positionY = 0
        
        if positionY > 130:
            positionY = 130
        

        if self.ball.vel.x < -20 and self.ball.pos.x > self.centroidDef.x + self.robotFunctions[self.indexRobot].getDefenderLine():
            vPrev = fabs(0.01 * (positionY - robotPos.y) / time_)

            if vPrev > vMaxPrevision:
                vPrev = vMaxPrevision
            
            elif vPrev < 0.4:
                vPrev = 0.4
            
            vDeltaPrev = vPrev * 1 #????

            if robotPos.x < self.centroidDef.x + self.robotFunctions[self.indexRobot].getDefenderLine() + 5 and robotPos.x > self.centroidDef.x + self.robotFunctions[self.indexRobot].getDefenderLine() - 5:
                self.prevGoal = Point2f(robotPos.x, positionY)
            
            else:
                self.prevGoal = Point2f(self.centroidDef.x + self.robotFunctions[self.indexRobot].getDefenderLine(), positionY)
            

            theta = angleTwoPoints(robotPos, self.prevGoal)
            self.alpha = theta - robotAngle
            self.alpha = ajustaAngulo(self.alpha)
            if fabs(self.alpha) <= limiarTheta:
                v = -vDeltaPrev * fabs(self.alpha) / limiarTheta + vPrev
                w = self.kp * self.alpha / 180 + self.kd * (self.alpha - self.lastAlpha)
                limiarTheta = 90 - deltaLimiar
            
            else:
                self.alpha = ajustaAngulo(self.alpha + 180)
                v = vDeltaPrev * fabs(self.alpha) / limiarTheta - vPrev
                w = self.kp * self.alpha / 180 + self.kd * (self.alpha - self.lastAlpha)
                limiarTheta = 90 + deltaLimiar
            

            if (fabs(robotPos.x - self.prevGoal.x) < 2) and (fabs(robotPos.y - self.prevGoal.y) < 2):
                v = 0
                if fabs(robotAngle) > 85 and fabs(robotAngle) < 95:
                    w = 0
                
                else:
                    self.alpha = 90 - robotAngle
                    self.alpha = ajustaAngulo(self.alpha)
                    if fabs(self.alpha) <= limiarTheta:
                        w = self.kp * self.alpha / 180
                    
                    else:
                        self.alpha = ajustaAngulo(self.alpha + 180)
                        w = self.kp * self.alpha / 180
                   
        elif self.ball.pos.x > self.centroidDef.x + self.robotFunctions[self.indexRobot].getDefenderLine() and robotPos.x < self.centroidDef.x + self.robotFunctions[self.indexRobot].getDefenderLine() + 7 and robotPos.x > self.centroidDef.x + self.robotFunctions[self.indexRobot].getDefenderLine() - 7:
            self.alpha = 90 - robotAngle
            self.alpha = ajustaAngulo(self.alpha)
            if fabs(self.alpha) <= limiarTheta:
                w = self.kp*self.alpha/180 + self.kd*(self.alpha - self.lastAlpha)
                limiarTheta = 90 - deltaLimiar
            
            else:
                self.alpha = ajustaAngulo(self.alpha+180)
                w = self.kp*self.alpha/180 + self.kd*(self.alpha - self.lastAlpha)
                limiarTheta = 90 + deltaLimiar
            

            if fabs(robotPos.y-self.ball.pos.y) < 3:
                v = 0
            
            elif robotAngle > 0 and self.ball.pos.y < robotPos.y:
                v = -vFollow
            
            elif robotAngle > 0 and self.ball.pos.y > robotPos.y:
                v = vFollow
            
            elif robotAngle < 0 and self.ball.pos.y < robotPos.y:
                v = vFollow
            
            elif robotAngle < 0 and self.ball.pos.y > robotPos.y:
                v = -vFollow
            
        else:
            if euclidean_dist(robotPos, self.robotFunctions[self.indexRobot].getGoal()) < 10:
               vMaxD = 0.4
            
            else:
                vMaxD = vMaxD
            
            vDelta = vMaxD*1
            theta = self.robotFunctions[self.indexRobot].getDirection()

            if self.inverte == True:
                if robotAngle > theta - 90 and robotAngle < theta + 90:
                    self.sentido = False #tras
                else:
                    self.sentido = True #frente
                
            
            self.alpha = theta - robotAngle
            self.alpha = ajustaAngulo(self.alpha)

            if self.inverte == False:
                if fabs(self.alpha) <= limiarTheta:
                    v = -vDelta * fabs(self.alpha) / limiarTheta + vMaxD
                    w = self.kp * self.alpha / 180 + self.kd * (self.alpha - self.lastAlpha)
                    limiarTheta = 90 - deltaLimiar
                
                else:
                    self.alpha = ajustaAngulo(self.alpha + 180)
                    v = vDelta * fabs(self.alpha) / limiarTheta - vMaxD
                    w = self.kp * self.alpha / 180 + self.kd * (self.alpha - self.lastAlpha)
                    limiarTheta = 90 + deltaLimiar
                
            
            else:
                if self.sentido == False:
                    self.alpha = ajustaAngulo(self.alpha + 180)
                    v = vDelta * fabs(self.alpha) / limiarTheta - vMaxD
                    w = self.kp * self.alpha / 180 + self.kd * (self.alpha - self.lastAlpha)
                    limiarTheta = 90 + deltaLimiar
                elif self.sentido == True:
                    v = -vDelta * fabs(self.alpha) / limiarTheta + vMaxD
                    w = self.kp * self.alpha / 180 + self.kd * (self.alpha - self.lastAlpha)
                    limiarTheta = 90 - deltaLimiar
                
            

            if (fabs(robotPos.x - self.robotFunctions[self.indexRobot].getGoal().x) < 4) and (fabs(robotPos.y - self.robotFunctions[self.indexRobot].getGoal().y) < 4):
                v = 0
                if fabs(robotAngle) > 85 and fabs(robotAngle) < 95:
                    w = 0
                
                else:
                    self.alpha = 90 - robotAngle
                    self.alpha = ajustaAngulo(self.alpha)
                    if fabs(self.alpha) <= limiarTheta:
                        w = self.kp*self.alpha/180
                    
                    else:
                        self.alpha = ajustaAngulo(self.alpha+180)
                        w = self.kp*self.alpha/180
        
        if self.airball:
            if (fabs(robotPos.x - self.robotFunctions[self.indexRobot].getGoal().x) < 4) and (fabs(robotPos.y - self.robotFunctions[self.indexRobot].getGoal().y) < 4):
                v = 0

                if self.ball.pos.y > self.centroidDef.y:
                    if robotAngle <=50 and robotAngle >= 40:
                        w = 0
                    
                    else:
                        self.alpha = 45 - robotAngle
                        self.alpha = ajustaAngulo(self.alpha)
                        if fabs(self.alpha) <= limiarTheta:
                            w = self.kp*self.alpha/180
                        
                        else:
                            self.alpha = ajustaAngulo(self.alpha+180)
                            w = self.kp*self.alpha/180
        
                else:
                    if robotAngle >= -50 and robotAngle <= -40:
                        w = 0
                    
                    else:
                        self.alpha = -45 - robotAngle
                        self.alpha = ajustaAngulo(self.alpha)
                        if fabs(self.alpha) <= limiarTheta:
                            w = self.kp*self.alpha/180
                        
                        else:
                            self.alpha = ajustaAngulo(self.alpha+180)
                            w = self.kp*self.alpha/180

        if w > 60:
            w = 60
        if w < -60:
            w = -60

        self.lVel = (v - w*self.l)*100
        self.rVel = (v + w*self.l)*100

        #print(f"lvel: {self.lvel}")
        #print(f"rvel: {self.rvel}")
        

        self.lastAlpha = self.alpha
        self.alphaS = self.alpha

        if euclidean_dist(self.ball.pos,robotPos) < distGiroGol:
            if robotPos.x < self.ball.pos.x:
                if robotPos.y > self.ball.pos.y:
                    self.lVel = -velGiroGol*100
                    self.rVel = velGiroGol*100
                
                elif robotPos.y < self.ball.pos.y:
                    self.lVel = velGiroGol*100
                    self.rVel = -velGiroGol*100
                
            
            else:
                if self.ball.pos.y > self.centroidDef.y + 40:
                    self.lVel = velGiroGol*100
                    self.rVel = -velGiroGol*100
                
                elif self.ball.pos.y < self.centroidDef.y - 40:
                    self.lVel = -velGiroGol*100
                    self.rVel = velGiroGol*100
                
    def striker(self):


        robotPos = self.teamRobot[self.indexRobot].getDataState().pos
        robotAngle = self.teamRobot[self.indexRobot].getDataState().angle
        vMax = self.vMax*0.01
        dist = euclidean_dist(robotPos,self.ball.pos)
        ct = 2
        #teempo = time.time() - self.clockAceleration

        if self.tempoTroca == 0:
            if self.temp == 0:
                self.posTemp = robotPos
                self.clockInvert = time.time()

            if euclidean_dist(robotPos,self.posTemp) <= 1:
                self.temp = time.time() - self.clockInvert
                if self.temp >= 3:
                    self.inverte = True
                    self.clockTroca = time.time()
                    self.temp = 0
                else :
                    self.inverte = False      
            else :
                self.inverte = False
                self.temp = 0
            
        if(self.inverte == True):
            self.tempoTroca = time.time() - self.clockTroca
            if self.tempoTroca > 7:
                self.tempoTroca = 0

        if dist < 25 and dist > 12 and robotPos.x < self.ball.pos.x:
            vMax = vMax*dist*(0.20)/(13)  #*0.3/16
            if vMax < 0.7:
                vMax = 0.7
        
    #-------Aceleração------
    #    vMax = self.vMax
    #    if (teempo<=ct)
    #   
    #        vMax=teempo*vMax/ct    

        limiarTheta = 90
        deltaLimiar = 30
        vDelta = 0.8*vMax

        #Angulo do CPU
        theta = self.robotFunctions[self.indexRobot].getDirection()

        #PID
        if self.inverte == True:
            if robotAngle > ajustaAngulo(theta) - 90 and robotAngle < ajustaAngulo(theta) + 90:
                self.sentido = False #tras
            else:
                self.sentido = True #frente
            
        self.alpha = theta - robotAngle
        self.alpha = ajustaAngulo(self.alpha)
        self.alphaS = self.alpha

        self.inverte = False
        if self.inverte == False:
            #if ((fabs(self.alpha) < 10) or (fabs(self.alpha) > 170)):
                #             self.kp = 20
                #             self.kd =      
            #else:
                #print("Curva")
            if (fabs(self.alpha) <= limiarTheta):
                v = -vDelta * fabs(self.alpha) / limiarTheta + vMax
                w = (self.kp * self.alpha / 180) + (self.kd * (self.alpha - self.lastAlpha))
                limiarTheta = 90 - deltaLimiar
            else:
                self.alpha = ajustaAngulo(self.alpha + 180)
                v = vDelta * fabs(self.alpha) / limiarTheta - vMax
                w = (self.kp * self.alpha / 180) + (self.kd * (self.alpha - self.lastAlpha))
                limiarTheta = 90 + deltaLimiar
        else:
            if(self.sentido == False):
                self.alpha = ajustaAngulo(self.alpha + 180)
                v = vDelta * fabs(self.alpha) / limiarTheta - vMax
                w = (self.kp * self.alpha / 180) + (self.kd * (self.alpha - self.lastAlpha))
                limiarTheta = 90 + deltaLimiar
            else: #if(self.sentido == True):
                v = -vDelta * fabs(self.alpha) / limiarTheta + vMax
                w = (self.kp * self.alpha / 180) + (self.kd * (self.alpha - self.lastAlpha))
                limiarTheta = 90 - deltaLimiar

        if w > 21:
            w = 21
        if w < -21:
            w = -21

        if self.robotFunctions[self.indexRobot].getStopOnGoal() == True:
            theta = self.robotFunctions[self.indexRobot].getDirection()
            self.alpha = theta - robotAngle
            self.alpha = ajustaAngulo(self.alpha)
            if(euclidean_dist(robotPos,self.robotFunctions[self.indexRobot].goal) < 25):
                vMax *= 0.6
                vDelta = vMax*0.8

            if fabs(self.alpha) <= limiarTheta:
                v = -vDelta * fabs(self.alpha) / limiarTheta + vMax
                w = (self.kp * self.alpha / 180) + (self.kd * (self.alpha - self.lastAlpha))
                limiarTheta = 90 - deltaLimiar
            
            else:
                self.alpha = ajustaAngulo(self.alpha + 180)
                v = vDelta * fabs(self.alpha) / limiarTheta - vMax
                w = (self.kp * self.alpha / 180) + (self.kd * (self.alpha - self.lastAlpha))
                limiarTheta = 90 + deltaLimiar
            
            if((fabs(robotPos.x - self.robotFunctions[self.indexRobot].getGoal().x) < 3) and (fabs(robotPos.y - self.robotFunctions[self.indexRobot].getGoal().y) < 3) ):
                v = 0
                if fabs(robotAngle) > 85 and fabs(robotAngle) < 95:
                    w = 0
                
                else:            
                    self.alpha = 90 - robotAngle
                    self.alpha = ajustaAngulo(self.alpha)

                    if fabs(self.alpha) <= limiarTheta:
                        w = 3*self.kp*self.alpha/180
                    else:
                        self.alpha = ajustaAngulo(self.alpha+180)
                        w = 3*self.kp*self.alpha/180
                    
        self.lVel = (v - w*self.l)*100
        self.rVel = (v + w*self.l)*100

        self.lastAlpha = self.alpha
        self.rotate()
    
        if self.robotFunctions[self.indexRobot].getAtkSituation() == True:
            if euclidean_dist(self.ball.pos,robotPos) < 10:
                if self.firstAceleration == True:
                    self.clockAceleration = time.time()
                
                self.firstAceleration = False
                self.atkSituation()
            else:
            
                self.firstAceleration = True

        else:
            self.firstAceleration = True

        #if self.robotFunctions[self.indexRobot].getKickState() == True:
        #    self.kickPenalty()

    def fake9(self):
        pass
    def midfield(self):
        pass
    def wing(self):
        pass
    def volante(self):
        pass
    def fuzzy(self):
        pass
    def impostor(self):
        pass
    def libero(self):
        pass
    def offdefender(self):
        pass
    def newstriker(self):
        pass    


    def setGains(self, mat: np.matrix):
        self.robotGains = mat

    def getGains(self):
        return self.robotGains


    def rotate(self):
        robotPos = self.teamRobot[self.indexRobot].getDataState().pos

        # Função para fazer o robô girar nos cantos
        if (self.ball.pos.y > (self.centroidAtk.y + 60)) and (euclidean_dist(self.ball.pos, robotPos) < self.distGiro):
            self.rVel = -self.velGiroLado*self.vMax
            self.lVel = self.velGiroLado*self.vMax
        
        elif (self.ball.pos.y < (self.centroidAtk.y - 60)) and (euclidean_dist(self.ball.pos,robotPos) < self.distGiro):
            self.rVel = self.velGiroLado*self.vMax
            self.lVel = -self.velGiroLado*self.vMax
        

        # Função para fazer o robô girar na linha de fundo de ataque
        if (self.ball.pos.y > (self.centroidAtk.y + 40)) and ((euclidean_dist(self.ball.pos, robotPos)) < self.distGiro) and (fabs(self.ball.pos.x - self.centroidAtk.x) < 15):
            self.rVel = -self.velGiroLado*self.vMax
            self.lVel = self.velGiroLado*self.vMax
        
        elif (self.ball.pos.y < (self.centroidAtk.y - 40)) and (euclidean_dist(self.ball.pos, robotPos) < self.distGiro) and (fabs(self.ball.pos.x - self.centroidAtk.x) < 15):
            self.rVel = self.velGiroLado*self.vMax
            self.lVel = -self.velGiroLado*self.vMax

        # Função para fazer o robô girar na linha de fundo de defesa
        if (self.ball.pos.y > (self.centroidDef.y + 35)) and ((euclidean_dist(self.ball.pos, robotPos)) < self.distGiro) and (fabs(self.ball.pos.x - self.centroidDef.x) < 15):
            self.rVel = -self.velGiroLado*self.vMax
            self.lVel = self.velGiroLado*self.vMax
        
        elif (self.ball.pos.y < (self.centroidDef.y - 35)) and (euclidean_dist(self.ball.pos, robotPos) < (self.distGiro and fabs(self.ball.pos.x - self.centroidDef.x) < 15)):
            self.rVel = self.velGiroLado*self.vMax
            self.lVel = -self.velGiroLado*self.vMax

    def rotateInv(self):
        robotPos = self.teamRobot[self.indexRobot].getDataState().pos
        # Função para fazer o robô girar nos cantos
        if (self.ball.pos.y > (self.centroidAtk.y + 60)) and (euclidean_dist(self.ball.pos, robotPos) < self.distGiro):
            self.rVel = self.velGiroLado*self.vMax
            self.lVel = -self.velGiroLado*self.vMax
        
        elif (self.ball.pos.y < (self.centroidAtk.y - 60)) and (euclidean_dist(self.ball.pos,robotPos) < self.distGiro):
            self.rVel = -self.velGiroLado*self.vMax
            self.lVel = self.velGiroLado*self.vMax
        

        # Função para fazer o robô girar na linha de fundo de ataque
        if (self.ball.pos.y > (self.centroidAtk.y + 40)) and ((euclidean_dist(self.ball.pos, robotPos)) < self.distGiro) and (fabs(self.ball.pos.x - self.centroidAtk.x) < 15):
            self.rVel = self.velGiroLado*self.vMax
            self.lVel = -self.velGiroLado*self.vMax
        
        elif (self.ball.pos.y < (self.centroidAtk.y - 40)) and (euclidean_dist(self.ball.pos, robotPos) < self.distGiro) and (fabs(self.ball.pos.x - self.centroidAtk.x) < 15):
            self.rVel = -self.velGiroLado*self.vMax
            self.lVel = self.velGiroLado*self.vMax
        

        # Função para fazer o robô girar na linha de fundo de defesa
        if (self.ball.pos.y > (self.centroidDef.y + 35)) and ((euclidean_dist(self.ball.pos, robotPos)) < self.distGiro) and (fabs(self.ball.pos.x - self.centroidDef.x) < 15):
            self.rVel = self.velGiroLado*self.vMax
            self.lVel = -self.velGiroLado*self.vMax
        
        elif (self.ball.pos.y < (self.centroidDef.y - 35)) and (euclidean_dist(self.ball.pos, robotPos) < (self.distGiro and fabs(self.ball.pos.x - self.centroidDef.x) < 15)):
            self.rVel = -self.velGiroLado*self.vMax
            self.lVel = self.velGiroLado*self.vMax

    def kickRotate(self):
        robotPos = self.teamRobot[self.indexRobot].getDataState().pos

        if (self.ball.pos.y > robotPos.y) and (euclidean_dist(self.ball.pos, robotPos) < self.distGiro):
            self.rVel = -self.velGiroLado
            self.lVel = self.velGiroLado
        
        elif (self.ball.pos.y < robotPos.y) and (euclidean_dist(self.ball.pos, robotPos) < self.distGiro):
            self.rVel = self.velGiroLado
            self.lVel = -self.velGiroLado

    def atkSituation(self):
        robotPos = self.teamRobot[self.indexRobot].getDataState().pos
        vAtk = 120*0.01
        robotAngle = self.teamRobot[self.indexRobot].getDataState().angle
        angBallRobot = robotAngle-angleTwoPoints(robotPos,self.ball.pos)
        angBallRobot = ajustaAngulo(angBallRobot)
        vTurn = 120*0.01

        if self.firstAceleration == False:
            if self.robotFunctions[self.indexRobot].getAtkSituationTiro() == True:
                if vAtk < 1:
                    vAtk = 1
            
            else:
                if self.vMax < 80:
                    if vAtk < 0.7:
                        vAtk = 0.7
                
                else:
                    if vAtk < self.vMax*0.01*0.8:
                        vAtk = self.vMax*0.01*0.8

        if euclidean_dist(self.ball.pos, robotPos) < 20:
            if fabs(robotAngle) < 90 and  fabs(angBallRobot) < 30:
                self.lVel = vAtk*100
                self.rVel = vAtk*100
            
            if fabs(angBallRobot) > 150:
                self.lVel = -vAtk*100
                self.rVel = -vAtk*100
            
            if euclidean_dist(robotPos, self.centroidAtk)<25 and self.robotFunctions[self.indexRobot].getlittleChute()==True:
                if robotPos.y > self.centroidAtk.y:
                    self.lVel = vTurn*100
                    self.rVel = -vTurn*100
                
                else:
                    self.lVel = -vTurn*100
                    self.rVel = vTurn*100
                
    def atkSituationInv(self):
        robotPos = self.teamRobot[self.indexRobot].getDataState().pos
        vAtk = 100*0.01
        robotAngle = self.teamRobot[self.indexRobot].getDataState().angle
        angBallRobot = robotAngle-angleTwoPoints(robotPos,self.ball.pos)
        angBallRobot = ajustaAngulo(angBallRobot)

        if euclidean_dist(self.ball.pos,robotPos) < 13:
            if fabs(robotAngle) < 90 and  fabs(angBallRobot) < 30:
                self.lVel = -vAtk
                self.rVel = -vAtk
            
            if fabs(angBallRobot) > 150:
                self.lVel = vAtk
                self.rVel = vAtk
            
    def setDirection(self, d:bool):
        self.robotDirection = d
    
    def setStrategy(self, s:int):
        self.strategy = s
    
    def getStrategy(self):
        return self.strategy
    
