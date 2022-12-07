#include "robot.h"

robot::robot()
{

}

void robot::setKp(double k)
{
    kp = k;
}

void robot::setKd(double k)
{
    kd = k;
}

void robot::setKi(double k)
{
    ki = k;
}


double robot::getKp()
{
    return kp;
}

double robot::getKd()
{
    return kd;
}

double robot::getKi()
{
    return ki;
}

dataState robot::getDataState()
{
    return kinetic;
}

int robot::getFunction()
{
    return function;
}

void robot::setFunction(int f)
{
    function = f;
}

void robot::setPosition(Point2f pos, double ang, Point2d vel)
{
    kinetic.pos = pos;
    kinetic.vel = vel;
    kinetic.angle = ang;
}

void robot::setQuadrant(int q)
{
    kinetic.quadrant = q;
}

void robot::cleanPath()
{
    pathPoints.clear();
}

void robot::AddPathPoint(Point2f point)
{
    pathPoints.push_back(point);
}

void robot::setTime(int time)
{
    this->time = time;
}

int robot::getTime()
{
    return time;
}

Point2f robot::getPosition()
{
    return kinetic.pos;
}

double robot::getAngle()
{
    return kinetic.angle;
}

Point2d robot::getVelocidade()
{
    return kinetic.vel;
}
