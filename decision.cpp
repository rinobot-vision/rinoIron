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

    int robotFunc[3];
    if(strategy == FIXED3)
    {
        teamRobot[2].setFunction(STRIKER);
        teamRobot[1].setFunction(DEFENDER);
        teamRobot[0].setFunction(GOALKEEPER);
    }

    else if(strategy == FIXED2_0)
    {
        teamRobot[2].setFunction(STRIKER);
        teamRobot[1].setFunction(LIBERO);
        teamRobot[0].setFunction(GOALKEEPER);
    }
    else if(strategy == FULL_ATK){
        robotFunc[0] = DEFENDER;
        if(ball.pos.x <= (centroidAtk.x + centroidDef.x)/2)
        {
            if(flagTrocaGS == true)
            {
                if(euclidean_dist(teamRobot[1].getDataState().pos, centroidDef) < euclidean_dist(teamRobot[2].getDataState().pos, centroidDef))
                {
                    robotFunc[1] = GOALKEEPER;
                    robotFunc[2] = STRIKER;
                }
                else
                {
                    robotFunc[1] = STRIKER;
                    robotFunc[2] = GOALKEEPER;
                }
                for(int i = 0; i <= 2; i++)
                {
                    if(robotFunc[i] == GOALKEEPER)
                    {
                        G = i;
                    }
                    if(robotFunc[i] == DEFENDER)
                    {
                        D = i;
                    }
                    if(robotFunc[i] == STRIKER)
                    {
                        S = i;
                    }
                }
                lastS = S;
                lastG = G;
                lastD = D;
            }
            else
            {
                timeTrocaGS = (float) (clock() - clockTrocaGS)/CLOCKS_PER_SEC;
                if(timeTrocaGS > 5)
                {
                    flagTrocaGS = true;
                }
                else
                {
                    robotFunc[lastS] = STRIKER;
                    robotFunc[lastD] = DEFENDER;
                    robotFunc[lastG] = GOALKEEPER;
                }
            }
        }

        else // bola no ataque
        {

            if(flagTrocaFS == true)
            {
                if(euclidean_dist(teamRobot[1].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[2].getDataState().pos, ball.pos))
                {
                    if(teamRobot[1].getDataState().pos.x < (ball.pos.x))
                    {
                        robotFunc[1] = STRIKER;
                        robotFunc[2] = FAKE9;
                    }
                    else if(teamRobot[2].getDataState().pos.x < (ball.pos.x))
                    {
                        robotFunc[1] = FAKE9;
                        robotFunc[2] = STRIKER;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
                else
                {
                    if(teamRobot[2].getDataState().pos.x < (ball.pos.x ))
                    {
                        robotFunc[1] = FAKE9;
                        robotFunc[2] = STRIKER;
                    }
                    else if(teamRobot[1].getDataState().pos.x < (ball.pos.x ))
                    {
                        robotFunc[1] = STRIKER;
                        robotFunc[2] = FAKE9;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
                for(int i = 0; i <= 2; i++)
                {
                    if(robotFunc[i] == FAKE9)
                    {
                        F = i;
                    }
                    if(robotFunc[i] == DEFENDER)
                    {
                        D = i;
                    }
                    if(robotFunc[i] == STRIKER)
                    {
                        S = i;
                    }
                }

                if(lastF != F)
                {
                    flagTrocaFS = false;
                    clockTrocaFS = clock();
                }

                lastF = F;
                lastD = D;
                lastS = S;
                lastM = F;
                lastW = S;
                tempT = 3;
            }

            if(CRUZAMENTO == true)
            {
                int indexMidfield;
                if(euclidean_dist(ball.pos, teamRobot[1].getDataState().pos) < euclidean_dist(ball.pos, teamRobot[2].getDataState().pos))
                {
                    indexMidfield = 2;
                }
                else
                {
                    indexMidfield = 1;
                }
                Point2f metaMidfield = Point2f(centroidAtk.x - 45, getCentroidAtk().y);
                if((euclidean_dist(teamRobot[indexMidfield].getDataState().pos, metaMidfield) < 25) && (ball.pos.y > centroidAtk.y + 30 || ball.pos.y < centroidAtk.y - 30) && (ball.pos.x > (centroidAtk.x - 25)) && (ball.pos.x >= 0))
                {
                    if(euclidean_dist(teamRobot[1].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[2].getDataState().pos, ball.pos))
                    {
                        robotFunc[1] = WING;
                        robotFunc[2] = MIDFIELD;
                        lastW = 1;
                        lastM = 2;
                    }
                    else
                    {
                        robotFunc[2] = WING;
                        robotFunc[1] = MIDFIELD;
                        lastW = 2;
                        lastM = 1;
                    }
                    flagTrocaFS = false;
                    flagCrossing = true;
                    clockTrocaFS = clock();
                    tempT = 2;

                }

            }

            if ( ball.pos.x < teamRobot[lastM].getDataState().pos.x)
            {
                if(flagCrossing)
                {
                    flagTrocaFS = true;
                }
            }

            if(flagTrocaFS == false)
            {
                timeTrocaFS = (float) (clock() - clockTrocaFS)/CLOCKS_PER_SEC;
                timeTrocaFS = timeTrocaFS * 10;

                if(timeTrocaFS > tempT)
                {
                    flagTrocaFS = true;
                    flagCrossing = false;
                }
                else
                {
                    if(flagCrossing)
                    {
                        robotFunc[lastW] = WING;
                        robotFunc[lastM] = MIDFIELD;
                        robotFunc[lastD] = DEFENDER;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastD] = DEFENDER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
            }

        }

        teamRobot[0].setFunction(robotFunc[0]);
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);

    }
    else if(strategy == FULL_ATK){
        robotFunc[0] = DEFENDER;
        if(ball.pos.x <= (centroidAtk.x + centroidDef.x)/2)
        {
            if(flagTrocaGS == true)
            {
                if(euclidean_dist(teamRobot[1].getDataState().pos, centroidDef) < euclidean_dist(teamRobot[2].getDataState().pos, centroidDef))
                {
                    robotFunc[1] = GOALKEEPER;
                    robotFunc[2] = STRIKER;
                }
                else
                {
                    robotFunc[1] = STRIKER;
                    robotFunc[2] = GOALKEEPER;
                }
                for(int i = 0; i <= 2; i++)
                {
                    if(robotFunc[i] == GOALKEEPER)
                    {
                        G = i;
                    }
                    if(robotFunc[i] == DEFENDER)
                    {
                        D = i;
                    }
                    if(robotFunc[i] == STRIKER)
                    {
                        S = i;
                    }
                }
                lastS = S;
                lastG = G;
                lastD = D;
            }
            else
            {
                timeTrocaGS = (float) (clock() - clockTrocaGS)/CLOCKS_PER_SEC;
                if(timeTrocaGS > 5)
                {
                    flagTrocaGS = true;
                }
                else
                {
                    robotFunc[lastS] = STRIKER;
                    robotFunc[lastD] = DEFENDER;
                    robotFunc[lastG] = GOALKEEPER;
                }
            }
        }

        else // bola no ataque
        {

            if(flagTrocaFS == true)
            {
                if(euclidean_dist(teamRobot[1].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[2].getDataState().pos, ball.pos))
                {
                    if(teamRobot[1].getDataState().pos.x < (ball.pos.x))
                    {
                        robotFunc[1] = STRIKER;
                        robotFunc[2] = FAKE9;
                    }
                    else if(teamRobot[2].getDataState().pos.x < (ball.pos.x))
                    {
                        robotFunc[1] = FAKE9;
                        robotFunc[2] = STRIKER;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
                else
                {
                    if(teamRobot[2].getDataState().pos.x < (ball.pos.x ))
                    {
                        robotFunc[1] = FAKE9;
                        robotFunc[2] = STRIKER;
                    }
                    else if(teamRobot[1].getDataState().pos.x < (ball.pos.x ))
                    {
                        robotFunc[1] = STRIKER;
                        robotFunc[2] = FAKE9;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
                for(int i = 0; i <= 2; i++)
                {
                    if(robotFunc[i] == FAKE9)
                    {
                        F = i;
                    }
                    if(robotFunc[i] == DEFENDER)
                    {
                        D = i;
                    }
                    if(robotFunc[i] == STRIKER)
                    {
                        S = i;
                    }
                }

                if(lastF != F)
                {
                    flagTrocaFS = false;
                    clockTrocaFS = clock();
                }

                lastF = F;
                lastD = D;
                lastS = S;
                lastM = F;
                lastW = S;
                tempT = 3;
            }

            if(CRUZAMENTO == true)
            {
                int indexMidfield;
                if(euclidean_dist(ball.pos, teamRobot[1].getDataState().pos) < euclidean_dist(ball.pos, teamRobot[2].getDataState().pos))
                {
                    indexMidfield = 2;
                }
                else
                {
                    indexMidfield = 1;
                }
                Point2f metaMidfield = Point2f(centroidAtk.x - 45, getCentroidAtk().y);
                if((euclidean_dist(teamRobot[indexMidfield].getDataState().pos, metaMidfield) < 25) && (ball.pos.y > centroidAtk.y + 30 || ball.pos.y < centroidAtk.y - 30) && (ball.pos.x > (centroidAtk.x - 25)) && (ball.pos.x >= 0))
                {
                    if(euclidean_dist(teamRobot[1].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[2].getDataState().pos, ball.pos))
                    {
                        robotFunc[1] = WING;
                        robotFunc[2] = MIDFIELD;
                        lastW = 1;
                        lastM = 2;
                    }
                    else
                    {
                        robotFunc[2] = WING;
                        robotFunc[1] = MIDFIELD;
                        lastW = 2;
                        lastM = 1;
                    }
                    flagTrocaFS = false;
                    flagCrossing = true;
                    clockTrocaFS = clock();
                    tempT = 2;

                }

            }

            if ( ball.pos.x < teamRobot[lastM].getDataState().pos.x)
            {
                if(flagCrossing)
                {
                    flagTrocaFS = true;
                }
            }

            if(flagTrocaFS == false)
            {
                timeTrocaFS = (float) (clock() - clockTrocaFS)/CLOCKS_PER_SEC;
                timeTrocaFS = timeTrocaFS * 10;

                if(timeTrocaFS > tempT)
                {
                    flagTrocaFS = true;
                    flagCrossing = false;
                }
                else
                {
                    if(flagCrossing)
                    {
                        robotFunc[lastW] = WING;
                        robotFunc[lastM] = MIDFIELD;
                        robotFunc[lastD] = DEFENDER;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastD] = DEFENDER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
            }

        }

        teamRobot[0].setFunction(robotFunc[0]);
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);

    }
    else if(strategy == NS)
    {
        int robotFunc[3];
        robotFunc[0] = GOALKEEPER;
        robotFunc[1] = DEFENDER;
        robotFunc[2] = STRIKER;
        float distG [3], distD[3], distS[3];
        Point2f PGol = centroidDef;

        if(ball.pos.x <= 85)
        {
            distD[0] = fabs(teamRobot[0].getDataState().pos.x - LineD);
            distD[1] = fabs(teamRobot[1].getDataState().pos.x - LineD);
            distD[2] = fabs(teamRobot[2].getDataState().pos.x - LineD);

            if(distD[0] < distD[1] && distD[0] < distD[2])
            {
                robotFunc[0] = DEFENDER;
                distG[1] = euclidean_dist(PGol, teamRobot[1].getDataState().pos);
                distG[2] = euclidean_dist(PGol, teamRobot[2].getDataState().pos);
                if (distG[1] < distG[2])
                {
                    robotFunc[1] = GOALKEEPER;
                    robotFunc[2] = STRIKER;
                }
                else if (distG[2] <= distG[1])
                {
                    robotFunc[2] = GOALKEEPER;
                    robotFunc[1] = STRIKER;
                }
            }
            else if(distD[1] < distD[0] && distD[1] < distD[2])
            {
                robotFunc[1] = DEFENDER;
                distG[0] = euclidean_dist(PGol, teamRobot[0].getDataState().pos);
                distG[2] = euclidean_dist(PGol, teamRobot[2].getDataState().pos);
                if (distG[0] < distG[2])
                {
                    robotFunc[0] = GOALKEEPER;
                    robotFunc[2] = STRIKER;
                }
                else if (distG[2] <= distG[0])
                {
                    robotFunc[2] = GOALKEEPER;
                    robotFunc[0] = STRIKER;
                }
            }
            else
            {
                robotFunc[2] = DEFENDER;
                distG[0] = euclidean_dist(PGol, teamRobot[0].getDataState().pos);
                distG[1] = euclidean_dist(PGol, teamRobot[1].getDataState().pos);
                if (distG[0] < distG[1])
                {
                    robotFunc[0] = GOALKEEPER;
                    robotFunc[1] = STRIKER;
                }
                else if (distG[1] <= distG[0])
                {
                    robotFunc[1] = GOALKEEPER;
                    robotFunc[0] = STRIKER;
                }
            }

            if(ball.pos.x < LineD )
            {
                for(int i = 0; i <= 2; i++)
                {
                    if(robotFunc[i] == GOALKEEPER)
                    {
                        G = i;
                    }
                    if(robotFunc[i] == DEFENDER)
                    {
                        D = i;
                    }
                    if(robotFunc[i] == STRIKER)
                    {
                        S = i;
                    }
                }

                if(ball.pos.y < centroidDef.y - 20)
                {
                    if(flagTrocaGS == true)
                    {
                        if(teamRobot[S].getDataState().pos.x > ball.pos.x ||  ( teamRobot[S].getDataState().pos.x < ball.pos.x && teamRobot[S].getDataState().pos.y > centroidDef.y - 20))
                        {
                            if(teamRobot[G].getDataState().pos.x > centroidDef.x + 3)
                            {
                                robotFunc[G] = DEFENDER;
                                lastD = G;
                                float DistS, DistD;
                                DistS = euclidean_dist(Point2f(centroidDef.x + 4, centroidDef.y - 15), teamRobot[S].getDataState().pos);
                                DistD = euclidean_dist(Point2f(centroidDef.x + 4, centroidDef.y - 15), teamRobot[D].getDataState().pos);

                                if (DistS < DistD)
                                {
                                    robotFunc[S] = GOALKEEPER;
                                    lastG = S;
                                    robotFunc[D] = STRIKER;
                                    lastS = D;
                                }
                                else
                                {
                                    robotFunc[D] = GOALKEEPER;
                                    lastG = D;
                                    robotFunc[S] = STRIKER;
                                    lastS = S;
                                }
                                clockTrocaGS = clock();
                                flagTrocaGS = false;
                            }
                        }
                    }
                }
                else if(ball.pos.y > centroidDef.y + 20)
                {
                    if(flagTrocaGS == true)
                    {
                        if(teamRobot[S].getDataState().pos.x > ball.pos.x || (teamRobot[S].getDataState().pos.x < ball.pos.x && teamRobot[S].getDataState().pos.y < centroidDef.y + 20))
                        {
                            if(teamRobot[G].getDataState().pos.x > centroidDef.x + 3)
                            {
                                robotFunc[G] = DEFENDER;
                                lastD = G;
                                float DistS, DistD;
                                DistS = euclidean_dist(Point2f(centroidDef.x + 4, centroidDef.y + 15), teamRobot[S].getDataState().pos);
                                DistD = euclidean_dist(Point2f(centroidDef.x + 4, centroidDef.y + 15), teamRobot[D].getDataState().pos);
                                if (DistS < DistD)
                                {
                                    robotFunc[S] = GOALKEEPER;
                                    lastG = S;
                                    robotFunc[D] = STRIKER;
                                    lastS = D;
                                }
                                else
                                {
                                    robotFunc[D] = GOALKEEPER;
                                    lastG = D;
                                    robotFunc[S] = STRIKER;
                                    lastS = S;
                                }
                                clockTrocaGS = clock();
                                flagTrocaGS = false;
                            }
                        }
                    }
                }
            }
            if(flagTrocaGS == false)
            {
                timeTrocaGS = (float) (clock() - clockTrocaGS)/CLOCKS_PER_SEC;
                if(timeTrocaGS > 10)
                {
                    flagTrocaGS = true;
                }
                else
                {
                    robotFunc[lastS] = STRIKER;
                    robotFunc[lastD] = DEFENDER;
                    robotFunc[lastG] = GOALKEEPER;
                }
            }
        }
        else if (ball.pos.x > 85)
        {
            distS[0] = euclidean_dist(ball.pos, teamRobot[0].getDataState().pos);
            distS[1] = euclidean_dist(ball.pos, teamRobot[1].getDataState().pos);
            distS[2] = euclidean_dist(ball.pos, teamRobot[2].getDataState().pos);

            if(flagTrocaFS == true)
            {
                if(distS[0] < distS[1] && distS[0] < distS[2])
                {
                    if(teamRobot[0].getDataState().pos.x < (ball.pos.x + 4.5))
                    {
                        robotFunc[0] = STRIKER;
                        distD[1] = fabs(LineD - teamRobot[1].getDataState().pos.x);
                        distD[2] = fabs(LineD - teamRobot[2].getDataState().pos.x);
                        if (distD[1] < distD[2])
                        {
                            robotFunc[1] = DEFENDER;
                            robotFunc[2] = FAKE9;
                        }
                        else if (distD[1] >= distD[2])
                        {
                            robotFunc[2] = DEFENDER;
                            robotFunc[1] = FAKE9;
                        }
                    }
                    else
                    {
                        if(distS[1] < distS[2])
                        {
                            robotFunc[1] = STRIKER;
                            robotFunc[0] = FAKE9;
                            robotFunc[2] = DEFENDER;
                        }
                        else
                        {
                            robotFunc[2] = STRIKER;
                            robotFunc[0] = FAKE9;
                            robotFunc[1] = DEFENDER;
                        }
                    }

                }
                else if(distS[1] < distS[0] && distS[1] < distS[2])
                {
                    if(teamRobot[1].getDataState().pos.x < (ball.pos.x + 4.5))
                    {
                        robotFunc[1] = STRIKER;
                        distD[0] = fabs(LineD - teamRobot[0].getDataState().pos.x);
                        distD[2] = fabs(LineD - teamRobot[2].getDataState().pos.x);
                        if (distD[0] < distD[2])
                        {
                            robotFunc[0] = DEFENDER;
                            robotFunc[2] = FAKE9;
                        }
                        else if (distD[0] >= distD[2])
                        {
                            robotFunc[2] = DEFENDER;
                            robotFunc[0] = FAKE9;
                        }
                    }
                    else
                    {
                        if(distS[0] < distS[2])
                        {
                            robotFunc[0] = STRIKER;
                            robotFunc[1] = FAKE9;
                            robotFunc[2] = DEFENDER;
                        }
                        else
                        {
                            robotFunc[2] = STRIKER;
                            robotFunc[1] = FAKE9;
                            robotFunc[0] = DEFENDER;
                        }
                    }
                }
                else
                {
                    if(teamRobot[2].getDataState().pos.x < (ball.pos.x + 4.5))
                    {
                        robotFunc[2] = STRIKER;
                        distD[1] = fabs(LineD - teamRobot[1].getDataState().pos.x);
                        distD[0] = fabs(LineD - teamRobot[0].getDataState().pos.x);
                        if (distD[0] < distD[1])
                        {
                            robotFunc[0] = DEFENDER;
                            robotFunc[1] = FAKE9;
                        }
                        else if (distD[0] >= distD[1])
                        {
                            robotFunc[1] = DEFENDER;
                            robotFunc[0] = FAKE9;
                        }
                    }
                    else
                    {
                        if(distS[0] < distS[1])
                        {
                            robotFunc[0] = STRIKER;
                            robotFunc[2] = FAKE9;
                            robotFunc[1] = DEFENDER;
                        }
                        else
                        {
                            robotFunc[1] = STRIKER;
                            robotFunc[2] = FAKE9;
                            robotFunc[0] = DEFENDER;
                        }
                    }
                }
                for(int i = 0; i <= 2; i++)
                {
                    if(robotFunc[i] == FAKE9)
                    {
                        F = i;
                    }
                    if(robotFunc[i] == DEFENDER)
                    {
                        D = i;
                    }
                    if(robotFunc[i] == STRIKER)
                    {
                        S = i;
                    }
                }
                lastF = F;
                lastD = D;
                lastS = S;
                lastW = S;
                lastM = F;
                flagTrocaFS = false;
                clockTrocaFS = clock();
                tempT = 2;
            }

            if(flagTrocaFS == false)
            {
                timeTrocaFS = (float) (clock() - clockTrocaFS)/CLOCKS_PER_SEC;
                if(timeTrocaFS > tempT)
                {
                    flagTrocaFS = true;
                    flagCrossing = false;
                }
                else
                {
                    if(flagCrossing == true)
                    {
                        robotFunc[lastW] = WING;
                        robotFunc[lastD] = DEFENDER;
                        robotFunc[lastM] = MIDFIELD;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastD] = DEFENDER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
            }

            if(ball.pos.y > centroidAtk.y + 45 || ball.pos.y < centroidAtk.y - 45)
            {
                if(euclidean_dist(teamRobot[S].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[F].getDataState().pos, ball.pos))
                {
                    robotFunc[S] = WING;
                    robotFunc[F] = MIDFIELD;
                    lastW = S;
                    lastM = F;
                }
                else
                {
                    robotFunc[F] = WING;
                    robotFunc[S] = MIDFIELD;
                    lastW = F;
                    lastM = S;
                }
                flagTrocaFS = false;
                flagCrossing = true;
                clockTrocaFS = clock();
                tempT = 5;
            }
        }
        teamRobot[0].setFunction(robotFunc[0]);
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);
    }

    else if(strategy == OFF)
    {
        robotFunc[0] = DEFENDER;
        if(ball.pos.x <= 85) {
            if(flagTrocaGS == true)
            {
                if(euclidean_dist(teamRobot[1].getDataState().pos, centroidDef) < euclidean_dist(teamRobot[2].getDataState().pos, centroidDef))
                {
                    robotFunc[1] = GOALKEEPER;
                    robotFunc[2] = STRIKER;
                }
                else
                {
                    robotFunc[1] = STRIKER;
                    robotFunc[2] = GOALKEEPER;
                }
                for(int i = 0; i <= 2; i++)
                {
                    if(robotFunc[i] == GOALKEEPER)
                    {
                        G = i;
                    }
                    if(robotFunc[i] == DEFENDER)
                    {
                        D = i;
                    }
                    if(robotFunc[i] == STRIKER)
                    {
                        S = i;
                    }
                }
                lastS = S;
                lastG = G;
                lastD = D;
            }
            if(flagTrocaGS == false)
            {
                timeTrocaGS = (float) (clock() - clockTrocaGS)/CLOCKS_PER_SEC;
                //                cout << "TimeTrocaGs: " << timeTrocaGS << endl;
                if(timeTrocaGS > 5)
                {
                    flagTrocaGS = true;
                }
                else
                {
                    robotFunc[lastS] = STRIKER;
                    robotFunc[lastD] = DEFENDER;
                    robotFunc[lastG] = GOALKEEPER;
                }
            }
        }
        else if (ball.pos.x > 85)
        {
            float distOFF1, distOFF2;
            distOFF1 = fabs(teamRobot[1].getDataState().pos.x - (centroidDef.x+ offLine));
            distOFF2 = fabs(teamRobot[2].getDataState().pos.x - (centroidDef.x+ offLine));
            float distRobot1Ball = euclidean_dist(teamRobot[1].getDataState().pos, ball.pos);
            float distRobot2Ball = euclidean_dist(teamRobot[2].getDataState().pos, ball.pos);
            if(ball.pos.x > (centroidDef.x+ offLine))
            {
                if(distRobot1Ball > distRobot2Ball)
                {
                    robotFunc[1] = OFFDEFENDER;
                    robotFunc[2] = STRIKER;
                }
                else
                {
                    robotFunc[2] = OFFDEFENDER;
                    robotFunc[1] = STRIKER;
                }
            }
            else
            {
                if(flagTrocaFS == true)
                {
                    if(euclidean_dist(teamRobot[1].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[2].getDataState().pos, ball.pos))
                    {
                        if(teamRobot[1].getDataState().pos.x < (ball.pos.x + 4.5))
                        {
                            robotFunc[1] = STRIKER;
                            robotFunc[2] = FAKE9;
                        }
                        else
                        {
                            robotFunc[1] = FAKE9;
                            robotFunc[2] = STRIKER;
                        }
                    }
                    else
                    {
                        if(teamRobot[2].getDataState().pos.x < (ball.pos.x + 4.5))
                        {
                            robotFunc[1] = FAKE9;
                            robotFunc[2] = STRIKER;
                        }
                        else
                        {
                            robotFunc[1] = STRIKER;
                            robotFunc[2] = FAKE9;
                        }
                    }
                    for(int i = 0; i <= 2; i++)
                    {
                        if(robotFunc[i] == FAKE9)
                        {
                            F = i;
                        }
                        if(robotFunc[i] == DEFENDER)
                        {
                            D = i;
                        }
                        if(robotFunc[i] == STRIKER)
                        {
                            S = i;
                        }
                    }

                    lastF = F;
                    lastD = D;
                    lastS = S;
                    flagTrocaFS = false;
                    clockTrocaFS = clock();
                    tempT = 3;
                }

                if(flagTrocaFS == false)
                {
                    timeTrocaFS = (float) (clock() - clockTrocaFS)/CLOCKS_PER_SEC;

                    if(timeTrocaFS > tempT)
                    {
                        flagTrocaFS = true;
                        flagCrossing = false;
                    }
                    else
                    {
                        robotFunc[lastS] = STRIKER;
                        robotFunc[lastD] = DEFENDER;
                        robotFunc[lastF] = FAKE9;
                    }
                }
            }
        }
        teamRobot[0].setFunction(robotFunc[0]);
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);
    }
}

