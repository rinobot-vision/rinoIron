from robot import *

FIXED3 = 0
FULL_ATK = 1
SAFE_FULL_ATK = 2
NS = 3

class Navigation:
    def __init__(self):
        self.de = float()
        self.deW = 2
        self.Kr = float()
        self.KrW = 5
        self.gSize = float()
        self.gSizeW = 0.6
        self.thetaDir = float()
        self.thePhi = float()
        self.alpha = float()
        self.k_larg = 0.055
        self.goal = Point2f()
        self.plotThePhi = [] #Point2f
        self.path = [] #int
        self.atkPaths = [] #Point2f

    def hyperbolicSpiral(self,robo:dataState):
        goal2 = Point2f(self.goal.x/5, self.goal.y/5)
        p = np.array([robo.pos.x/5,robo.pos.y/5,1])
        p = np.matrix(p).T

        m_trans = np.matrix([[ 1, 0, -goal2.x],[ 0, 1, -goal2.y],[ 0, 0, 1]])
        m_rot = np.matrix([[cos(-self.thetaDir),-sin(-self.thetaDir),0],[sin(-self.thetaDir),cos(-self.thetaDir),0],[0,0,1]])

        ph = m_rot*m_trans*p

        thetaUp = atan2((ph[1]-self.de-self.gSize),ph[0]) + self.thetaDir
        thetaDown = atan2((ph[1]+self.de+self.gSize),ph[0]) + self.thetaDir
        rhoUp = sqrt(pow(ph[0],2) + pow((ph[1]-self.de-self.gSize),2))
        rhoDown = sqrt(pow(ph[0],2) + pow((ph[1]+self.de+self.gSize),2))

        if ph[1] > self.gSize:
            phi = thetaUp + pi*(2-(self.de+self.Kr)/(rhoUp+self.Kr))/2
        elif ph[1] < -self.gSize:
            phi = thetaDown - pi*(2-(self.de+self.Kr)/(rhoDown+self.Kr))/2
        else:
            phi = self.thetaDir

        return phi
        
    def hyperbolicSpiral2(self, robo:dataState):
        p = np.array([robo.pos.x, robo.pos.y, 1])
        p = np.matrix(p).T

        m_trans1 = np.matrix([[ 1, 0, -self.goal.x], [0, 1, -self.goal.y], [0, 0, 1]])
        m_trans2 = np.matrix([[ 1, 0, self.goal.x], [0, 1, self.goal.y], [0, 0, 1]])
        m_rot = np.matrix([[ cos(-self.thetaDir), -sin(-self.thetaDir), 0],[ sin(-self.thetaDir), cos(-self.thetaDir), 0],[ 0, 0, 1]])

        ph = m_trans2 * m_rot * m_trans1 * p

        pl = [ph[0], ph[1] - self.de]
        pr = [ph[0], ph[1] + self.de]
        y_aux = ph[1] - self.goal.y
        yl = y_aux + self.de
        yr = y_aux - self.de
        rho = sqrt(pow(pl[0] - self.goal.x, 2) + pow(pl[1] - self.goal.y, 2))
        theta = atan2(pl[1] - self.goal.y, pl[0] - self.goal.x)

        if rho > self.de:
            phi_ccw = theta + pi * (2 - ((self.de + self.Kr) / (rho + self.Kr))) / 2
        
        else:
            phi_ccw = theta + pi * sqrt(rho / self.de) / 2
        
        rho = sqrt(pow(pr[0] - self.goal.x, 2) + pow(pr[1] - self.goal.y, 2))
        theta = atan2(pr[1] - self.goal.y, pr[0] - self.goal.x)

        if rho > self.de:
            phi_cw = theta - pi * (2 - ((self.de + self.Kr) / (rho + self.Kr))) / 2
        
        else:        
            phi_cw = theta - pi * sqrt(rho / self.de) / 2
        
        vec = [(yl*cos(phi_ccw) - yr*cos(phi_cw))/(2*self.de), (yl*sin(phi_ccw) - yr*sin(phi_cw))/(2*self.de)]

        phi = atan2(vec[1], vec[0]) + self.thetaDir
        return phi

    def repulsiveMath(self, robo:dataState, obj:Point2f):
        rot_angle = pi/2
        k_const = 1 # k_larg: Quanto menor mais desvia [0.1 , 0.01]
        m = (self.goal.y-obj.y)/(self.goal.x-obj.x)

        norm = sqrt(pow(obj.x - robo.pos.x,2) + pow(obj.y - robo.pos.y,2))
        k_const = self.k_larg*norm

        if obj.x <= self.goal.x:
            if(robo.pos.y-obj.y > m*(robo.pos.x-obj.x)):
                a = -1
            else:
                a = 1
        
        else:
            if(robo.pos.y-obj.y > m*(robo.pos.x-obj.x)):
                a = 1
            else:
                a = -1
        
        vec_out = Point2f(self.goal.x - robo.pos.x, self.goal.y - robo.pos.y)

        rot = np.matrix([[cos(a*rot_angle), sin(a*rot_angle)], [-sin(a*rot_angle), cos(a*rot_angle)]])

        vec_aux = np.matrix([[ vec_out.x],[ vec_out.y]])
        vec_tan_aux = rot*vec_aux
        vec_tan = Point2f(vec_tan_aux(0),vec_aux(1))

        vec = Point2f(vec_tan.x + k_const*vec_out.x, vec_tan.y + k_const*vec_out.y)

        psi = atan2(vec.y,vec.x)

        return psi*180/pi
    
    def univectorField(self, robo:dataState, enemy:Point2f):
        self.thePhi = ajustaAngulo(self.hyperbolicSpiral(robo)*180/pi)

    def univectorFieldForPlot(self, robo:dataState , enemy:Point2f):
        return ajustaAngulo(self.hyperbolicSpiral(robo)*180/pi)

    def GaussianFunc(self, r:float):
        delta = self.Kr
        if (r < 0):
            r = 0
        G = pow(e, (-pow(r, 2) / (2 * pow(delta, 2))))
        return G

    def repulsiveAngle(self, p1:Point2f, p2:Point2f):
        
        if (p1.x - p2.x < 0):
            alpha = atan((p1.y - p2.y) / (p1.x - p2.x)) + pi
        else:
            alpha = atan((p1.y - p2.y) / (p1.x - p2.x))

        return alpha
    
    def AddPlotPoint(self, point:Point2f):
        self.plotThePhi.append(point)

    def AddAtkPoint(self, point:Point2f ):
        self.atkPaths.append(point)

 