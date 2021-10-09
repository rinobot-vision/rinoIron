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
        int robotFunc[5];
        teamRobot[4].setFunction(DEFENDER2);
        teamRobot[1].setFunction(DEFENDER);
        teamRobot[0].setFunction(GOALKEEPER);
        if(ball.pos.x < 125)
        {
            teamRobot[3].setFunction(FAKE9);
            teamRobot[2].setFunction(STRIKER);
        }
        else
        {
            if(flagTrocaFS == true)
            {
                if (euclidean_dist(teamRobot[2].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[3].getDataState().pos, ball.pos))
                {
                    if(teamRobot[2].getDataState().pos.x < (ball.pos.x))
                    {
                        robotFunc[2] = STRIKER;
                        robotFunc[3] = FAKE9;
                    }
                    else if(teamRobot[3].getDataState().pos.x < (ball.pos.x))
                    {
                        robotFunc[2] = FAKE9;
                        robotFunc[3] = STRIKER;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
                else
                {
                    if(teamRobot[3].getDataState().pos.x < (ball.pos.x))
                    {
                        robotFunc[3] = STRIKER;
                        robotFunc[2] = FAKE9;
                    }
                    else if(teamRobot[2].getDataState().pos.x < (ball.pos.x))
                    {
                        robotFunc[3] = FAKE9;
                        robotFunc[2] = STRIKER;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastF] = FAKE9;
                    }
                }

                if(robotFunc[2] == FAKE9)
                {
                    F = 2;
                    S = 3;
                }
                if(robotFunc[2] == STRIKER)
                {
                    S = 2;
                    F = 3;
                }

                if(lastF != F)
                {
                    flagTrocaFS = false;
                    clockTrocaFS = clock();
                }

                lastF = F;
                lastS = S;
                tempT = 3;

            }
            else if(flagTrocaFS == false)
            {
                timeTrocaFS = (float) (clock() - clockTrocaFS)/CLOCKS_PER_SEC;
                timeTrocaFS = timeTrocaFS * 10;

                if(timeTrocaFS > tempT)
                {
                    flagTrocaFS = true;
                }
                else
                {
                    robotFunc[lastS] = STRIKER;
                    robotFunc[lastF] = FAKE9;
                }
            }
            teamRobot[2].setFunction(robotFunc[2]);
            teamRobot[3].setFunction(robotFunc[3]);
        }
    }
    cout<< "robo 2: "<< teamRobot[2].getFunction()<<endl;
    cout<< "robo 3: "<< teamRobot[3].getFunction()<<endl;

}
