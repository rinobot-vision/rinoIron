#include "navigation.h"

Navigation::Navigation()
{
    gSizeW = 0.60;
    deW = 2;
    KrW = 0.5;
    k_larg = 0.055;
}

Navigation::~Navigation(){

}

void Navigation::univectorField(dataState robo, Point2f enemy)
{
    thePhi = ajustaAngulo(hyperbolicSpiral(robo)*180/PI);
}

float Navigation::univectorFieldForPlot(dataState robo, Point2f enemy)
{
    return ajustaAngulo(hyperbolicSpiral(robo)*180/PI);
}

float Navigation::GaussianFunc(float r)
{
    float delta = Kr;
    float G;
    if (r < 0)
        r = 0;
    G = pow(M_E, (-pow(r, 2) / (2 * pow(delta, 2))));
    return G;
}

float Navigation::repulsiveAngle(Point2f p1, Point2f p2)
{
    float alpha;
    if (p1.x - p2.x < 0)
        alpha = atan((p1.y - p2.y) / (p1.x - p2.x)) + PI;
    else
        alpha = atan((p1.y - p2.y) / (p1.x - p2.x));

    return alpha;
}

float Navigation::hyperbolicSpiral(dataState robo)
{
    float thetaUp,thetaDown,rhoUp,rhoDown;
    float phi;
    Point2f goal2;
    goal2.x = goal.x/5;
    goal2.y = goal.y/5;
    Vector3d p(robo.pos.x/5,robo.pos.y/5,1),ph(0,0,0);

    MatrixXd m_trans(3,3),m_rot(3,3);
    m_trans  << 1, 0, -goal2.x, 0, 1, -goal2.y, 0, 0, 1;
    m_rot << cos(-thetaDir),-sin(-thetaDir),0,sin(-thetaDir),cos(-thetaDir),0,0,0,1;

    ph = m_rot*m_trans*p;

    thetaUp = atan2((ph(1)-de-gSize),ph(0)) + thetaDir;
    thetaDown = atan2((ph(1)+de+gSize),ph(0)) + thetaDir;
    rhoUp = sqrt(pow(ph(0),2) + pow((ph(1)-de-gSize),2));
    rhoDown = sqrt(pow(ph(0),2) + pow((ph(1)+de+gSize),2));

    if (ph(1)>gSize)
        phi = thetaUp + PI*(2-(de+Kr)/(rhoUp+Kr))/2;
    else if (ph(1)< -gSize)
        phi = thetaDown - PI*(2-(de+Kr)/(rhoDown+Kr))/2;
    else
        phi = thetaDir;

    return phi;
}

float Navigation::hyperbolicSpiral2(dataState robo)
{
    float phi;
    float theta, rho, y_aux, yl, yr, phi_cw, phi_ccw;
    float pl[2], pr[2], vec[2];

    Vector3d p(robo.pos.x, robo.pos.y, 1), ph(0, 0, 0);

    Matrix3d m_trans1(3, 3), m_trans2(3, 3), m_rot(3, 3);
    m_trans1 << 1, 0, -goal.x, 0, 1, -goal.y, 0, 0, 1;
    m_trans2 << 1, 0, goal.x, 0, 1, goal.y, 0, 0, 1;
    m_rot << cos(-thetaDir), -sin(-thetaDir), 0, sin(-thetaDir), cos(-thetaDir), 0, 0, 0, 1;

    ph = m_trans2 * m_rot * m_trans1 * p;

    pl[0] = ph(0);
    pl[1] = ph(1) - de;
    pr[0] = ph(0);
    pr[1] = ph(1) + de;
    y_aux = ph(1) - goal.y;
    yl = y_aux + de;
    yr = y_aux - de;
    rho = sqrt(pow(pl[0] - goal.x, 2) + pow(pl[1] - goal.y, 2));
    theta = atan2(pl[1] - goal.y, pl[0] - goal.x);

    if (rho > de)
    {
        phi_ccw = theta + PI * (2 - ((de + Kr) / (rho + Kr))) / 2;
    }
    else
    {
        phi_ccw = theta + PI * sqrt(rho / de) / 2;
    }

    rho = sqrt(pow(pr[0] - goal.x, 2) + pow(pr[1] - goal.y, 2));
    theta = atan2(pr[1] - goal.y, pr[0] - goal.x);

    if (rho > de)
    {
        phi_cw = theta - PI * (2 - ((de + Kr) / (rho + Kr))) / 2;
    }
    else
    {
        phi_cw = theta - PI * sqrt(rho / de) / 2;
    }

    vec[0] = (yl*cos(phi_ccw) - yr*cos(phi_cw))/(2*de);
    vec[1] = (yl*sin(phi_ccw) - yr*sin(phi_cw))/(2*de);

    phi = atan2(vec[1], vec[0]) + thetaDir;
    return phi;
}

float Navigation::repulsiveMath(dataState robo, Point2f obj)
{
    float rot_angle = PI/2;
    float k_const = 1; // k_larg: Quanto menor mais desvia [0.1 , 0.01]
    float m = (goal.y-obj.y)/(goal.x-obj.x);
    float norm;
    float psi;
    int a;
    Point2f vec_out, vec_tan, vec;
    MatrixXd rot(2,2);
    MatrixXd vec_tan_aux(2,1), vec_aux(2,1);

    norm = sqrt(pow(obj.x - robo.pos.x,2) + pow(obj.y - robo.pos.y,2));
    k_const = k_larg*norm;

    if(obj.x <= goal.x)
    {
        if(robo.pos.y-obj.y > m*(robo.pos.x-obj.x))
            a = -1;
        else
            a = 1;
    }
    else
    {
        if(robo.pos.y-obj.y > m*(robo.pos.x-obj.x))
            a = 1;
        else
            a = -1;
    }

    vec_out.x = goal.x - robo.pos.x;
    vec_out.y = goal.y - robo.pos.y;

    rot << cos(a*rot_angle), sin(a*rot_angle), -sin(a*rot_angle), cos(a*rot_angle);

    vec_aux << vec_out.x, vec_out.y;
    vec_tan_aux = rot*vec_aux;
    vec_tan.x = vec_tan_aux(0);
    vec_tan.y = vec_tan_aux(1);

    vec.x = vec_tan.x + k_const*vec_out.x;
    vec.y = vec_tan.y + k_const*vec_out.y;

    psi = atan2(vec.y,vec.x);

    return psi*180/PI;
}

void Navigation::AddPlotPoint(Point2f point)
{
    plotThePhi.push_back(point);
}

void Navigation::AddAtkPoint(Point2f point)
{
    atkPaths.push_back(point);
}
