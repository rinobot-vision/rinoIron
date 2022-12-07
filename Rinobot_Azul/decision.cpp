#include "decision.h"

Decision::Decision()
{
    changePermission = false;
    firstTime = true;
    fuzzy.start();
    FuzzyImpostor.start();
    fuzzydownbelow.start();
    fuzzy_torneio.start();

}

Decision::~Decision()
{

}

vector<robot> Decision::getTeamRobots()
{
    return teamRobot;
}

Point2d Decision::getCentroidDef()
{
    return centroidDef;
}

Point2d Decision::getCentroidAtk()
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

void Decision::setAreas(Point2d def, Point2d atk)
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
        robotFunc[2] = GOALKEEPER;
        robotFunc[0] = STRIKER;
        robotFunc[1] = DEFENDER;

        teamRobot[2].setFunction(GOALKEEPER);
        if(swapRoles){
            teamRobot[1].setFunction(DEFENDER);
            teamRobot[0].setFunction(STRIKER);
        }
        else{
            teamRobot[1].setFunction(DEFENDER);
            teamRobot[0].setFunction(STRIKER);
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
            Point2d defenderPoint;
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
    if(strategy == FUZZY_){
        //cout << "ENTROU NA FUZZY" << endl;
        robotFunc[0] = GOALKEEPER;
        teamRobot[0].setFunction(GOALKEEPER);
        Point2f ballAux = Point2f(ball.pos.x - 10, ball.pos.y);
        if(ballAux.x < 0){
            ballAux.x = 0;
        }
        if(ballAux.x > 150){
            ballAux.x = 150;
        }
        Point2f robotAux = Point2f(teamRobot[lastCompanheiroFuzzy].getDataState().pos.x - 10, teamRobot[lastCompanheiroFuzzy].getDataState().pos.y);
        if(robotAux.x < 0){
            robotAux.x = 0;
        }
        if(robotAux.x > 150){
            robotAux.x = 150;
        }

        float ballVel = ball.vel.x;

        if ( ballVel > 1){
            ballVel = 1;
        }
        if ( ballVel < -1){
            ballVel = -1;
        }
        Point3f retuFuzzy = fuzzy.encontraDestino(ballAux, robotAux, ballVel);
        cout << "TROCA = " << retuFuzzy.z << endl;
        if (retuFuzzy.z > 0.5){
            if (lastCompanheiroFuzzy == 1){
                lastCompanheiroFuzzy = 2;
                robotFunc[2] = STRIKER;
                robotFunc[1] = FUZZYROBOT;
            }
            else{
                lastCompanheiroFuzzy = 1;
                robotFunc[1] = STRIKER;
                robotFunc[2] = FUZZYROBOT;
            }
        }
        else
        {
            if (lastCompanheiroFuzzy == 1){
                robotFunc[1] = STRIKER;
                robotFunc[2] = FUZZYROBOT;
            }
            else{
                robotFunc[2] = STRIKER;
                robotFunc[1] = FUZZYROBOT;
            }
        }
        if(robotFunc[1] == FUZZYROBOT){
            cout << "ROBO 1: FUNÇÃO FUZZY "<< endl;
        }
        if(robotFunc[2] == FUZZYROBOT){
            cout << "ROBO 2: FUNÇÃO FUZZY "<< endl;
        }
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);
    }
    if(strategy == KNN_)
    {
        knnfunction.loadFunctions(teamRobot, ball);
    }
    if(strategy == IMPOSTOR_){
        //cout << "ENTROU NA IMPOSTOR" << endl;
        int quadranteRobo[3];
        int ordemQuadrante[3];
        int ordemDaOrdemQuadrante[3];
        for(int i = 0; i < 3; i ++){
            if(teamRobot[i].getDataState().pos.x <= 65){
                if(teamRobot[i].getDataState().pos.y <= 32.5){
                    quadranteRobo[i] = 1;
                }
                if(teamRobot[i].getDataState().pos.y <= 65 && teamRobot[i].getDataState().pos.y > 32.5){
                    quadranteRobo[i] = 2;
                }
                if(teamRobot[i].getDataState().pos.y <= 97.5 && teamRobot[i].getDataState().pos.y > 65){
                    quadranteRobo[i] = 3;
                }
                if(teamRobot[i].getDataState().pos.y <= 130 && teamRobot[i].getDataState().pos.y > 97.5){
                    quadranteRobo[i] = 4;
                }
            }
            if(teamRobot[i].getDataState().pos.x <= 105 && teamRobot[i].getDataState().pos.x > 65){
                if(teamRobot[i].getDataState().pos.y <= 32.5){
                    quadranteRobo[i] = 5;
                }
                if(teamRobot[i].getDataState().pos.y <= 65 && teamRobot[i].getDataState().pos.y > 32.5){
                    quadranteRobo[i] = 6;
                }
                if(teamRobot[i].getDataState().pos.y <= 97.5 && teamRobot[i].getDataState().pos.y > 65){
                    quadranteRobo[i] = 7;
                }
                if(teamRobot[i].getDataState().pos.y <= 130 && teamRobot[i].getDataState().pos.y > 97.5){
                    quadranteRobo[i] = 8;
                }
            }
            if(teamRobot[i].getDataState().pos.x <= 160 && teamRobot[i].getDataState().pos.x > 105){
                if(teamRobot[i].getDataState().pos.y <= 32.5){
                    quadranteRobo[i] = 9;
                }
                if(teamRobot[i].getDataState().pos.y <= 65 && teamRobot[i].getDataState().pos.y > 32.5){
                    quadranteRobo[i] = 10;
                }
                if(teamRobot[i].getDataState().pos.y <= 97.5 && teamRobot[i].getDataState().pos.y > 65){
                    quadranteRobo[i] = 11;
                }
                if(teamRobot[i].getDataState().pos.y <= 130 && teamRobot[i].getDataState().pos.y > 97.5){
                    quadranteRobo[i] = 12;
                }
            }
        }
        int aux = 0;
        for(int i = 0; i < 3; i++){
            ordemQuadrante[i] = -1;
        }
        if (quadranteRobo[0] == quadranteRobo[1] && quadranteRobo[0] == quadranteRobo[2])
        {
            ordemQuadrante[0] = 0;
            ordemQuadrante[1] = 1;
            ordemQuadrante[2] = 2;
        }
        else
        {
            for(int i = 0; i < 3; i++){
                for(int j = 0; j <3 ; j++){
                    if( i == j ){
                        continue;
                    }
                    if(quadranteRobo[i] <= quadranteRobo[j]){
                        aux++;
                    }
                }
                if(aux == 2){
                    if(ordemQuadrante[0] == -1){
                        ordemQuadrante[0] = i;
                    }
                    else
                    {
                        ordemQuadrante[1] = i;
                    }
                }
                if(aux == 1){
                    if(ordemQuadrante[1] == -1){
                        ordemQuadrante[1] = i;
                    }
                    else{
                        ordemQuadrante[2] = i;
                    }
                }
                if(aux == 0){
                    if(ordemQuadrante[2] == -1){
                        ordemQuadrante[2] = i;
                    }
                    else
                    {
                        ordemQuadrante[1] = i;
                    }
                }
                aux = 0;
            }
        }
        Point2f ballPos = Point2f(ball.pos.x -10,ball.pos.y);
        if(ballPos.x < 0)
            ballPos.x = 1;
        Point2f roboA = Point2f(teamRobot[ordemQuadrante[0]].getDataState().pos.x - 10,teamRobot[ordemQuadrante[0]].getDataState().pos.y);
        if(roboA.x < 0)
            roboA.x = 1;
        Point2f roboB = Point2f(teamRobot[ordemQuadrante[1]].getDataState().pos.x - 10,teamRobot[ordemQuadrante[1]].getDataState().pos.y);
        if(roboB.x < 0)
            roboB.x = 1;
        Point2f roboC = Point2f(teamRobot[ordemQuadrante[2]].getDataState().pos.x - 10,teamRobot[ordemQuadrante[2]].getDataState().pos.y);
        if(roboC.x < 0)
            roboC.x = 1;

        Point3f retuFuzzy = FuzzyImpostor.encontraDestino(ballPos, roboA, roboB,roboC);
        float func[3] = {retuFuzzy.x, retuFuzzy.y, retuFuzzy.z};
        //cout << "Return fuzzy: " << retuFuzzy << endl;
//        cout << "Quadrantes : (" ;
//        for(int i = 0; i < 3; i ++){
//            cout << quadranteRobo[i] << ",";
//        }

//        cout << ")" << endl;
//        cout << "RoboA: " << roboA << endl;
//        cout << "RoboA: " << roboB << endl;
//        cout << "RoboA: " << roboC << endl;
//        cout << "Bola: " << ballPos << endl;

        for(int i = 0; i < 3; i ++){
            if(func[i] <= 0,5 ){
                robotFunc[ordemQuadrante[i]] = GOALKEEPER;
            }
            if(func[i] > 0.5 && func[i] <= 1.5){
                robotFunc[ordemQuadrante[i]] = DEFENDER;
            }
            if(func[i] > 1.5 && func[i] <= 2.5){
                robotFunc[ordemQuadrante[i]] = STRIKER;
            }
            if(func[i] > 2.5 && func[i] <= 3.5){
                robotFunc[ordemQuadrante[i]] = FAKE9;
            }
            if(func[i] > 3.5 && func[i] <= 4.5){
                robotFunc[ordemQuadrante[i]] = IMPOSTOR;
            }
        }
//        for(int i = 0; i < 3; i++){
//            cout << "Robo " << i << ": " << robotFunc[i] << endl;
//        }
        teamRobot[0].setFunction(robotFunc[0]);
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);
    }
    if(strategy == DOWNBELOW){
        //cout << "ENTROU NA DOWNBELOW" << endl;
        robotFunc[0] = GOALKEEPER;
        teamRobot[0].setFunction(GOALKEEPER);
        Point2f ballAux = Point2f(ball.pos.x - 10, ball.pos.y);
        if(ballAux.x < 0){
            ballAux.x = 0;
        }
        if(ballAux.x > 150){
            ballAux.x = 150;
        }
        Point2f robotAux = Point2f(teamRobot[lastCompanheiroFuzzy].getDataState().pos.x - 10, teamRobot[lastCompanheiroFuzzy].getDataState().pos.y);
        if(robotAux.x < 0){
            robotAux.x = 0;
        }
        if(robotAux.x > 150){
            robotAux.x = 150;
        }

        float ballVel = ball.vel.x;

        if ( ballVel > 1){
            ballVel = 1;
        }
        if ( ballVel < -1){
            ballVel = -1;
        }

        float valorTroca = fuzzydownbelow.decideTroca(ballAux, robotAux);
//        cout << "TROCA = " << valorTroca << endl;
        if (valorTroca > 0.5){
            if (lastCompanheiroFuzzy == 1){
                lastCompanheiroFuzzy = 2;
                robotFunc[2] = STRIKER;
                robotFunc[1] = FUZZYROBOT;
            }
            else{
                lastCompanheiroFuzzy = 1;
                robotFunc[1] = STRIKER;
                robotFunc[2] = FUZZYROBOT;
            }
        }
        else
        {
            if (lastCompanheiroFuzzy == 1){
                robotFunc[1] = STRIKER;
                robotFunc[2] = FUZZYROBOT;
            }
            else{
                robotFunc[2] = STRIKER;
                robotFunc[1] = FUZZYROBOT;
            }
        }
//        if(robotFunc[1] == FUZZYROBOT){
//            cout << "ROBO 1: FUNÇÃO FUZZY "<< endl;
//        }
//        if(robotFunc[2] == FUZZYROBOT){
//            cout << "ROBO 2: FUNÇÃO FUZZY "<< endl;
//        }
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);
    }
    if(strategy == FUZZYT){
        //cout << "ENTROU NA FUZZY" << endl;
        robotFunc[0] = GOALKEEPER;
        teamRobot[0].setFunction(GOALKEEPER);
        Point2f ballAux = Point2f(ball.pos.x - 10, ball.pos.y);
        if(ballAux.x < 0){
            ballAux.x = 0;
        }
        else if(ballAux.x > 150){
            ballAux.x = 150;
        }
        Point2f strikerAux = Point2f(teamRobot[strikerIndex].getDataState().pos.x - 10, teamRobot[strikerIndex].getDataState().pos.y);
        if(strikerAux.x < 0){
            strikerAux.x = 0;
        }
        else if(strikerAux.x > 150){
            strikerAux.x = 150;
        }

        Point2f liberoAux = Point2f(teamRobot[liberoIndex].getDataState().pos.x - 10, teamRobot[liberoIndex].getDataState().pos.y);
        if(liberoAux.x < 0){
            liberoAux.x = 0;
        }
        else if(liberoAux.x > 150){
            liberoAux.x = 150;
        }

        float ballVel = ball.vel.x;

        if ( ballVel > 1){
            ballVel = 1;
        }
        else if ( ballVel < -1){
            ballVel = -1;
        }
        Point3f retuFuzzy = fuzzy_torneio.encontraDestino(ballAux, strikerAux, liberoAux);
        if (retuFuzzy.z > 0.5){
            if (strikerIndex == 1){
                 strikerIndex= 2;
                 robotFunc[2] = NEWSTRIKER;
                 robotFunc[1] = LIBERO;
             }
             else{
                 strikerIndex = 1;
                robotFunc[2] = LIBERO;
                robotFunc[1] = NEWSTRIKER;
            }
        }
        else
        {
            if (strikerIndex == 1){
                robotFunc[1] = NEWSTRIKER;
                robotFunc[2] = LIBERO;
            }
            else{
                robotFunc[1] = LIBERO;
                robotFunc[2] = NEWSTRIKER;
            }
        }
//        if(robotFunc[1] == FUZZYROBOT){
//           // cout << "ROBO 1: FUNÇÃO FUZZY "<< endl;
//        }
//        if(robotFunc[2] == FUZZYROBOT){
//           // cout << "ROBO 2: FUNÇÃO FUZZY "<< endl;
//        }
        teamRobot[1].setFunction(robotFunc[1]);
        teamRobot[2].setFunction(robotFunc[2]);

//        cout<<"Valor de troca: "<< retuFuzzy.z<<endl;
        cout<<"0:"<< robotFunc[0]<<"    1: "<<robotFunc[1]<<"   2: "<<robotFunc[2]<<endl;
    }
}

