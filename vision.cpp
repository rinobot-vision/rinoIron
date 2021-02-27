#include "vision.h"

vision::vision()
{

}

vision::~vision()
{

}

void vision::run()
{
    emit emit_info(robots,enemy,centroidDef,centroidAtk,ball);
}

void vision::setCentroidDef(Point2f def)
{
    centroidDef = def;
}

void vision::setCentroidAtk(Point2f atk)
{
    centroidAtk = atk;
}

void vision::setRobots(vector<robot> robotsPoints)
{

    robots = robotsPoints;
}

void vision::setBall(dataState ball)
{
    this->ball = ball;
}

vector<robot> vision::getRobots()
{
    return robots;
}

vector<robot> vision::getEnemy()
{
    return enemy;
}


Point2f vision::getCentroidDef()
{
    return centroidDef;
}

Point2f vision::getCentroidAtk()
{
    return centroidAtk;
}

dataState vision::getBall()
{
    return ball;
}

