#include "decision.h"

Decision::Decision()
{
    changePermission = false;
    firstTime = true;
}

Decision::~Decision()
{

}

vector<robot> Decision::getTeamRobots()
{
    return teamRobot;
}

Point2f Decision::getCentroidDef()
{
    return centroidDef;
}

Point2f Decision::getCentroidAtk()
{
    return centroidAtk;
}

dataState Decision::getBall()
{
    return ball;
}

void Decision::setRobots(vector<robot> r)
{
    teamRobot = r;
}

void Decision::setAreas(Point2f def, Point2f atk)
{
    centroidDef = def;
    centroidAtk = atk;
}

void Decision::setBall(dataState b)
{
    ball = b;
}

void Decision::setStrategy(int s)
{
    strategy = s;
}

void Decision::setKnnInformation(KNN k)
{
    knnfunction = k;
}

int Decision::getStrategy()
{
    return strategy;
}

void Decision::updateObjectives()
{
    if(strategy == FIXED5)
    {
        teamRobot[4].setFunction(DEFENDERSHIELD);
        teamRobot[3].setFunction(FAKE9);
        teamRobot[2].setFunction(GOALKEEPER);
        teamRobot[1].setFunction(DEFENDER);
        teamRobot[0].setFunction(STRIKER);
    }
}

