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
        //cout << "ENTROU NA FIXA" << endl;
        robotFunc[1] = GOALKEEPER;
        robotFunc[2] = STRIKER;
        robotFunc[0] = DEFENDER;

        teamRobot[1].setFunction(GOALKEEPER);
        if(swapRoles){
            teamRobot[0].setFunction(DEFENDER);
            teamRobot[2].setFunction(STRIKER);
        }
        else{
            teamRobot[0].setFunction(DEFENDER);
            teamRobot[2].setFunction(STRIKER);
        }
    }
    if(strategy == FULL_ATK){
        //cout << "ENTROU NA FULL ATK" << endl;
        robotFunc[0] = DEFENDER;
//        cout<<"Ang robo 0: " << teamRobot[0].getDataState().angle<<endl;
//        cout<<"Ang robo 1: " << teamRobot[1].getDataState().angle<<endl;
//        cout<<"Ang robo 2: " << teamRobot[2].getDataState().angle<<"\n"<<endl;
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
                lastM = F;
                lastW = S;
                flagTrocaFS = false;
                clockTrocaFS = clock();
                tempT = 3;
            }

            if((ball.pos.y > centroidAtk.y + 40 || ball.pos.y < centroidAtk.y - 40) && (ball.pos.x > (centroidAtk.x - 35)))
            {
                //                cout<<"Entrou no cruzamento"<<endl;
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
                tempT = 6;

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
                //cout << "TimeTrocaFs: " << timeTrocaFS << endl;

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
    if(strategy == SAFE_FULL_ATK){
        cout << "ENTROU NA SAFE FULL ATK" << endl;
        int robotFunc[3];
        robotFunc[0] = GOALKEEPER;
        if(ball.pos.x < (centroidAtk.x + centroidDef.x)/2)
        {
            Point2f defenderPoint;
            defenderPoint.y = ball.pos.y;
            defenderPoint.x = 48;
            if(fabs(teamRobot[1].getDataState().pos.x - LineD) < fabs(teamRobot[2].getDataState().pos.x - LineD))
            {
                if(euclidean_dist(teamRobot[1].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[2].getDataState().pos, ball.pos))
                {
                    robotFunc[1] = DEFENDER;
                    robotFunc[2] = STRIKER;
                }
                else
                {
                    robotFunc[1] = STRIKER;
                    robotFunc[2] = DEFENDER;
                }
            }
            else
            {
                if(euclidean_dist(teamRobot[2].getDataState().pos, ball.pos) < euclidean_dist(teamRobot[1].getDataState().pos, ball.pos))
                {
                    robotFunc[1] = STRIKER;
                    robotFunc[2] = DEFENDER;
                }
                else
                {
                    robotFunc[1] = DEFENDER;
                    robotFunc[2] = STRIKER;
                }
            }
        }
        else
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

            //Crossing
        }
        teamRobot[0].setFunction(robotFunc[0]);
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);
    }
    if(strategy == NS)
    {
        cout << "ENTROU NA NS" << endl;
        int robotFunc[3];
        robotFunc[0] = GOALKEEPER;
        robotFunc[1] = DEFENDER;
        robotFunc[2] = STRIKER;
        float distG0, distG1, distG2, distD0, distD1, distD2, distS0, distS1, distS2, distM0, distM1, distM2;
        Point2f PGol = centroidDef;
        Point2f PM;
        float distFake9 = 30;

        if(ball.pos.x <= 85)
        {
            distD0 = fabs(teamRobot[0].getDataState().pos.x - LineD);
            distD1 = fabs(teamRobot[1].getDataState().pos.x - LineD);
            distD2 = fabs(teamRobot[2].getDataState().pos.x - LineD);

            if(distD0 < distD1 && distD0 < distD2)
            {
                robotFunc[0] = DEFENDER;
                distG1 = euclidean_dist(PGol, teamRobot[1].getDataState().pos);
                distG2 = euclidean_dist(PGol, teamRobot[2].getDataState().pos);
                if (distG1 < distG2)
                {
                    robotFunc[1] = GOALKEEPER;
                    robotFunc[2] = STRIKER;
                }
                else if (distG2 <= distG1)
                {
                    robotFunc[2] = GOALKEEPER;
                    robotFunc[1] = STRIKER;
                }
            }
            else if(distD1 < distD0 && distD1 < distD2)
            {
                robotFunc[1] = DEFENDER;
                distG0 = euclidean_dist(PGol, teamRobot[0].getDataState().pos);
                distG2 = euclidean_dist(PGol, teamRobot[2].getDataState().pos);
                if (distG0 < distG2)
                {
                    robotFunc[0] = GOALKEEPER;
                    robotFunc[2] = STRIKER;
                }
                else if (distG2 <= distG0)
                {
                    robotFunc[2] = GOALKEEPER;
                    robotFunc[0] = STRIKER;
                }
            }
            else if(distD2 < distD0 && distD2 < distD1)
            {
                robotFunc[2] = DEFENDER;
                distG0 = euclidean_dist(PGol, teamRobot[0].getDataState().pos);
                distG1 = euclidean_dist(PGol, teamRobot[1].getDataState().pos);
                if (distG0 < distG1)
                {
                    robotFunc[0] = GOALKEEPER;
                    robotFunc[1] = STRIKER;
                }
                else if (distG1 <= distG0)
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
                                //cout << "Trocou" << endl;
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
                                //cout << "Trocou" << endl;
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
                cout << "TimeTrocaGs: " << timeTrocaGS << endl;
                if(timeTrocaGS > 20)
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
            distS0 = euclidean_dist(ball.pos, teamRobot[0].getDataState().pos);
            distS1 = euclidean_dist(ball.pos, teamRobot[1].getDataState().pos);
            distS2 = euclidean_dist(ball.pos, teamRobot[2].getDataState().pos);

            if(flagTrocaFS == true)
            {
                if(distS0 < distS1 && distS0 < distS2)
                {
                    if(teamRobot[0].getDataState().pos.x < (ball.pos.x + 4.5))
                    {
                        robotFunc[0] = STRIKER;

                        PM.x = teamRobot[0].getDataState().pos.x - distFake9;

                        distM1 = fabs(LineD - teamRobot[1].getDataState().pos.x);
                        distM2 = fabs(LineD - teamRobot[2].getDataState().pos.x);
                        if (distM1 < distM2)
                        {
                            robotFunc[1] = DEFENDER;
                            robotFunc[2] = FAKE9;
                        }
                        else if (distM1 >= distM2)
                        {
                            robotFunc[2] = DEFENDER;
                            robotFunc[1] = FAKE9;
                        }
                    }
                    else
                    {
                        if(distS1 < distS2)
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
                else if(distS1 < distS0 && distS1 < distS2)
                {
                    if(teamRobot[1].getDataState().pos.x < (ball.pos.x + 4.5))
                    {
                        robotFunc[1] = STRIKER;
                        PM.x = teamRobot[1].getDataState().pos.x - distFake9;
                        distM0 = fabs(LineD - teamRobot[0].getDataState().pos.x);
                        distM2 = fabs(LineD - teamRobot[2].getDataState().pos.x);
                        if (distM0 < distM2)
                        {
                            robotFunc[0] = DEFENDER;
                            robotFunc[2] = FAKE9;
                        }
                        else if (distM0 >= distM2)
                        {
                            robotFunc[2] = DEFENDER;
                            robotFunc[0] = FAKE9;
                        }
                    }
                    else
                    {
                        if(distS0 < distS2)
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
                else if(distS2 < distS0 && distS2 < distS1)
                {
                    if(teamRobot[2].getDataState().pos.x < (ball.pos.x + 4.5))
                    {
                        robotFunc[2] = STRIKER;
                        PM.x = teamRobot[2].getDataState().pos.x - distFake9;
                        distM1 = fabs(LineD - teamRobot[1].getDataState().pos.x);
                        distM0 = fabs(LineD - teamRobot[0].getDataState().pos.x);
                        if (distM0 < distM1)
                        {
                            robotFunc[0] = DEFENDER;
                            robotFunc[1] = FAKE9;
                        }
                        else if (distM0 >= distM1)
                        {
                            robotFunc[1] = DEFENDER;
                            robotFunc[0] = FAKE9;
                        }
                    }
                    else
                    {
                        if(distS0 < distS1)
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
                flagTrocaFS = false;
                clockTrocaFS = clock();
                tempT = 2;
            }

            if(flagTrocaFS == false)
            {
                timeTrocaFS = (float) (clock() - clockTrocaFS)/CLOCKS_PER_SEC;
                //cout << "TimeTrocaFs: " << timeTrocaFS << endl;
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
    if(strategy == OFF)
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
                        //cout << "TimeTrocaFs: " << timeTrocaFS << endl;

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

