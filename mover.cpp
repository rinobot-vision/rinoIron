#include "mover.h"

Mover::Mover()
{
    kp = 20;
    kd = 2;
}

Mover::~Mover()
{
    mutex.lock();
    mutex.unlock();
}

void Mover::run()
{
    if((int)teamRobot.size() > indexRobot)
    {
        //updateGains();
        switch (teamRobot[indexRobot].getFunction())
        {
        case GOALKEEPER:
            goalkeeper();
            break;
        case DEFENDER:
            defender();
            break;
        case STRIKER:
            striker();
            break;
        case FAKE9:
            fake9();
            break;
        case MIDFIELD:
            midfield();
            break;
        case WING:
            wing();
            break;
        case VOLANTE:
            volante();
            break;
        case OFFDEFENDER:
            offdefender();
            break;
        default:
            break;
        }
    }
}

void Mover::setVMax(float v)
{
    vMax = v;
}

void Mover::setRobots(vector<robot> r)
{
    teamRobot = r;
}

void Mover::setAreas(Point2f def, Point2f atk)
{
    if(!againstTheTeam)
    {
        centroidDef = def;
        centroidAtk = atk;
    }
    else
    {
        centroidDef = atk;
        centroidAtk = def;
    }
}

void Mover::setAgainstTheTeam(bool a)
{
    againstTheTeam = a;
}

void Mover::setBall(dataState b)
{
    ball = b;
}

float Mover::getVMax()
{
    return vMax;
}

float Mover::getLVel()
{
    return lVel;
}

float Mover::getRVel()
{
    return rVel;
}

void Mover::setGameFunctions(GameFunctions *g0, GameFunctions *g1, GameFunctions *g2)
{
    robotFunctions[0] = g0;
    robotFunctions[1] = g1;
    robotFunctions[2] = g2;
}

void Mover::goalkeeper()
{
    float vFollow = 0.6;
    float vMaxPrevision = 1.2;
    float limiarTheta = 90;
    float deltaLimiar = 30;
    float vMaxGol = 0.7;

    // Case 8
    //    vFollow = 0.50;
    //    vMaxGol = 0.75;

    // Case Fast Goalkepper
    //    vFollow = 0.75;
    //    vMaxGol = vMax*0.01;

    float vDeltaGol = vMaxGol;
    float distGiroGol = 8;
    float velGiroGol = 1.0;
    float vPrev, vDeltaPrev;

    float v, w, theta;

    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float robotVel = sqrt(pow(teamRobot[indexRobot].getDataState().vel.x,2) + pow(teamRobot[indexRobot].getDataState().vel.y,2))/100;
    float robotOmega = teamRobot[indexRobot].getDataState().omega;

    float time = (robotPos.x+5 - ball.pos.x)/ball.vel.x;
    float positionY = ball.pos.y + time*ball.vel.y;

    if(positionY < 45)
    {
        positionY = 45;
    }
    if(positionY > 85)
    {
        positionY = 85;
    }

    prevGoal = robotPos;

    if (ball.vel.x < -10 && (positionY > centroidDef.y-30) && (positionY < centroidDef.y+30))
    {
        vPrev = fabs(0.01 * (positionY - robotPos.y) / time);

        if (vPrev > vMaxPrevision)
        {
            vPrev = vMaxPrevision;
        }
        else if (vPrev < 0.5)
        {
            vPrev = 0.5;
        }
        vDeltaPrev = vPrev * 1;

        if(robotPos.x < centroidDef.x + 5 && robotPos.x > centroidDef.x - 5)
        {
            prevGoal = Point2f(robotPos.x, positionY);
        }
        else
        {
            prevGoal = Point2f(centroidDef.x, positionY);
        }

        theta = angleTwoPoints(robotPos, prevGoal);
        alpha = theta - robotAngle;
        alpha = ajustaAngulo(alpha);
        if (fabs(alpha) <= limiarTheta)
        {
            v = -vDeltaPrev * fabs(alpha) / limiarTheta + vPrev;
            w = kp * alpha / 180 + kd * (alpha - lastAlpha);
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDeltaPrev * fabs(alpha) / limiarTheta - vPrev;
            w = kp * alpha / 180 + kd * (alpha - lastAlpha);
            limiarTheta = 90 + deltaLimiar;
        }

        if ((fabs(robotPos.x - prevGoal.x) < 5) && (fabs(robotPos.y - prevGoal.y) < 4))
        {
            v = 0;
            if (fabs(robotAngle) > 85 && fabs(robotAngle) < 95)
            {
                w = 0;
            }
            else
            {
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    //                    w = 2* kp * alpha / 180;
                    w = kp*alpha/180 + kd*(alpha - lastAlpha);
                }
                else
                {
                    alpha = ajustaAngulo(alpha + 180);
                    //                    w = 2 * kp * alpha / 180;
                    w = kp*alpha/180 + kd*(alpha - lastAlpha);
                }
            }
        }
//                  cout << "PrevY: " << positionY << endl;
//                  cout << "ballX: " << ball.pos.x << endl;
//                  cout << "Prevision vel: " << v << endl;
    }
    else
    {
        if ((ball.pos.x < centroidDef.x + 70) && (ball.pos.y > centroidDef.y - 25) && (ball.pos.y < centroidDef.y + 25) && (robotPos.x < centroidDef.x+5) && (robotPos.x >= centroidDef.x -5))
        {
            theta = robotFunctions[indexRobot]->getDirection();
            alpha = theta - robotAngle;
            alpha = ajustaAngulo(alpha);
            if (fabs(alpha) <= limiarTheta)
            {
                w = kp*alpha/180 + kd*(alpha - lastAlpha);
                limiarTheta = 90 - deltaLimiar;
            }
            else
            {
                alpha = ajustaAngulo(alpha+180);
                w = kp*alpha/180 + kd*(alpha - lastAlpha);
                limiarTheta = 90 + deltaLimiar;
            }

            if(fabs(robotPos.y-ball.pos.y) < 3)
            {
                v = 0;
            }
            else if ((robotAngle > 0) && (ball.pos.y < robotPos.y))
            {
                v = -vFollow;
            }
            else if ((robotAngle > 0) && (ball.pos.y > robotPos.y))
            {
                v = vFollow;
            }
            else if ((robotAngle < 0) && (ball.pos.y < robotPos.y))
            {
                v = vFollow;
            }
            else if ((robotAngle < 0) && (ball.pos.y > robotPos.y))
            {
                v = -vFollow;
            }
                                cout << "Follow: " << v << endl;

        }
        else
        {
              cout << "Return:" << endl;
            theta = robotFunctions[indexRobot]->getDirection();
//            cout << "ANGULO: " << theta << endl;
            alpha = theta - robotAngle;
            alpha = ajustaAngulo(alpha);

            if (fabs(alpha) <= limiarTheta)
            {
                v = -vDeltaGol*fabs(alpha)/limiarTheta + vMaxGol;
                w = kp*alpha/180 + kd*(alpha - lastAlpha);
                limiarTheta = 90 - deltaLimiar;
            }
            else
            {
                alpha = ajustaAngulo(alpha+180);
                v = vDeltaGol*fabs(alpha)/limiarTheta - vMaxGol;
                w = kp*alpha/180 + kd*(alpha - lastAlpha);
                limiarTheta = 90 + deltaLimiar;
            }

            //if((robotPos.x - robotFunctions[indexRobot]->getGoal().x < 3) && (robotPos.x - robotFunctions[indexRobot]->getGoal().x >= 0) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 4) )
            if ((fabs(robotPos.x - robotFunctions[indexRobot] ->getGoal().x <2.5)) && (((robotFunctions[indexRobot]->getGoal().y - robotPos.y<4) && (ball.pos.y>65)) || ((robotPos.y - robotFunctions[indexRobot]->getGoal().y <4) && (ball.pos.y<65))))
            {
                v = 0;
                if(fabs(robotAngle) > 85 && fabs(robotAngle) < 95)
                {
                    w = 0;
                    // cout << "chegou" << endl;
                }
                else
                {
                    //                    cout << "ajusta" << endl;
                    alpha = 90 - robotAngle;
                    alpha = ajustaAngulo(alpha);
                    if (fabs(alpha) <= limiarTheta)
                    {
                        w = kp*alpha/180;
                        // w = kp*alpha/180 + kd*(alpha - lastAlpha);
                    }
                    else
                    {
                        alpha = ajustaAngulo(alpha+180);
                        w = kp*alpha/180;
                        //w = kp*alpha/180 + kd*(alpha - lastAlpha);
                    }
                }
            }
            // cout<< "vel = "<<v<< "  w = "<<w<<endl;
        }
    }



    if(w > 15)
        w = 15;
    if(w < -15)
        w = -15;

    //    if(w < 8)
    //        w = 8;

    //    if(w > -8)
    //        w = 8;


    lVel = (v - w*l)*100;
    rVel = (v + w*l)*100;

//    cout << "lVel: " << lVel << " | rVel: " << rVel << endl;

    lastAlpha = alpha;
    alphaS = alpha;

    if((robotPos.y > ball.pos.y) && (euclidean_dist(ball.pos,robotPos) < distGiroGol))
    {
        lVel = -velGiroGol*100;
        rVel = velGiroGol*100;
    }
    else if((robotPos.y < ball.pos.y) && (euclidean_dist(ball.pos,robotPos) < distGiroGol))
    {
        lVel = velGiroGol*100;
        rVel = -velGiroGol*100;
    }

    if(robotFunctions[indexRobot]->getTiroMeta() == true)
    {
        kickGoalKeeper();
    }
    // cout<<"Vel = "<<v;
}


void Mover::defender()
{

    float vMaxD = 0.65;
    float vFollow = 0.50;
    float vMaxPrevision = 0.85;
    float vDelta = 1*vMaxD;
    float limiarTheta = 90;
    float deltaLimiar = 30;
    float vPrev, vDeltaPrev;
    float distGiroGol = 10;
    float velGiroGol = 1.0;

    airball = false;

    if(ball.pos.x < centroidDef.x +15 && ball.pos.y < centroidDef.y + 40 && ball.pos.y > centroidDef.y - 40 && againstTheTeam == false ){
        airball = true;
    }

    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float robotVel = sqrt(pow(teamRobot[indexRobot].getDataState().vel.x,2) + pow(teamRobot[indexRobot].getDataState().vel.y,2))/100;

    float v,w,theta;

    if(tempoTroca == 0){

        if(temp == 0)
        {
            posTemp = robotPos;
            clockInvert = clock();
        }
        //cout<<"distancia: "<<euclidean_dist(robotPos,posTemp)<<endl;
        if(euclidean_dist(robotPos,posTemp) <= 0.2)
        {
            temp = (float) (clock() - clockInvert)/CLOCKS_PER_SEC;
            if(temp >= 3){
                //cout<<"tempo"<<endl;
                inverte = true;
                clockTroca = clock();
                temp = 0;
            }else {
                inverte = false;
            }
        }else {
            inverte = false;
            temp = 0;
        }
    }
    if(inverte == true)
    {
        tempoTroca = (float) (clock() - clockTroca)/CLOCKS_PER_SEC;
        if(tempoTroca > 3)
            tempoTroca = 0;
    }

    float time = (robotPos.x+5 - ball.pos.x)/ball.vel.x;
    float positionY = ball.pos.y + time*ball.vel.y;

    if(positionY < 0)
    {
        positionY = 0;
    }
    if(positionY > 130)
    {
        positionY = 130;
    }

    if (ball.vel.x < -20 && ball.pos.x > centroidDef.x + robotFunctions[indexRobot]->getDefenderLine())
    {
        vPrev = fabs(0.01 * (positionY - robotPos.y) / time);

        if (vPrev > vMaxPrevision)
        {
            vPrev = vMaxPrevision;
        }
        else if (vPrev < 0.4)
        {
            vPrev = 0.4;
        }
        vDeltaPrev = vPrev * 1;

        if(robotPos.x < centroidDef.x + robotFunctions[indexRobot]->getDefenderLine() + 5 && robotPos.x > centroidDef.x + robotFunctions[indexRobot]->getDefenderLine() - 5)
        {
            prevGoal = Point2f(robotPos.x, positionY);
        }
        else
        {
            prevGoal = Point2f(centroidDef.x + robotFunctions[indexRobot]->getDefenderLine(), positionY);
        }

        theta = angleTwoPoints(robotPos, prevGoal);
        alpha = theta - robotAngle;
        alpha = ajustaAngulo(alpha);
        if (fabs(alpha) <= limiarTheta)
        {
            v = -vDeltaPrev * fabs(alpha) / limiarTheta + vPrev;
            w = kp * alpha / 180 + kd * (alpha - lastAlpha);
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDeltaPrev * fabs(alpha) / limiarTheta - vPrev;
            w = kp * alpha / 180 + kd * (alpha - lastAlpha);
            limiarTheta = 90 + deltaLimiar;
        }

        if ((fabs(robotPos.x - prevGoal.x) < 2) && (fabs(robotPos.y - prevGoal.y) < 2))
        {
            v = 0;
            if (fabs(robotAngle) > 85 && fabs(robotAngle) < 95)
            {
                w = 0;
            }
            else
            {
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = 3 * kp * alpha / 180;
                }
                else
                {
                    alpha = ajustaAngulo(alpha + 180);
                    w = 3 * kp * alpha / 180;
                }
            }
        }

        //          cout << "PrevY: " << positionY << endl;
        //          cout << "ballY: " << ball.pos.y << endl;
        //                  cout << "Prevision vel: " << v << endl;
    }

    else if (ball.pos.x > centroidDef.x + robotFunctions[indexRobot]->getDefenderLine() && robotPos.x < centroidDef.x + robotFunctions[indexRobot]->getDefenderLine() + 7 && robotPos.x > centroidDef.x + robotFunctions[indexRobot]->getDefenderLine() - 7)
    {
        alpha = 90 - robotAngle;
        alpha = ajustaAngulo(alpha);
        if (fabs(alpha) <= limiarTheta)
        {
            w = kp*alpha/180 + kd*(alpha - lastAlpha);
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha+180);
            w = kp*alpha/180 + kd*(alpha - lastAlpha);
            limiarTheta = 90 + deltaLimiar;
        }

        if(fabs(robotPos.y-ball.pos.y) < 3)
        {
            v = 0;
        }
        else if (robotAngle > 0 && ball.pos.y < robotPos.y)
        {
            v = -vFollow;
        }
        else if (robotAngle > 0 && ball.pos.y > robotPos.y)
        {
            v = vFollow;
        }
        else if (robotAngle < 0 && ball.pos.y < robotPos.y)
        {
            v = vFollow;
        }
        else if (robotAngle < 0 && ball.pos.y > robotPos.y)
        {
            v = -vFollow;
        }
        //        cout << "Follow" << endl;
    }
    else
    {
        //    cout << "Return" << endl;
        if (euclidean_dist(robotPos, robotFunctions[indexRobot]->getGoal()) < 10)
        {
            vMaxD = 0.4;
        }
        else
        {
            vMaxD = vMaxD;
        }
        vDelta = vMaxD*1;
        theta = robotFunctions[indexRobot]->getDirection();

        if(inverte == true)
        {
            if(robotAngle > theta - 90 && robotAngle < theta + 90)
                sentido = false; //tras
            else {
                sentido = true; //frente
            }
        }
        alpha = theta - robotAngle;
        alpha = ajustaAngulo(alpha);

        if(inverte == false)
        {
            if (fabs(alpha) <= limiarTheta)
            {
                v = -vDelta * fabs(alpha) / limiarTheta + vMaxD;
                w = kp * alpha / 180 + kd * (alpha - lastAlpha);
                limiarTheta = 90 - deltaLimiar;
            }
            else
            {
                alpha = ajustaAngulo(alpha + 180);
                v = vDelta * fabs(alpha) / limiarTheta - vMaxD;
                w = kp * alpha / 180 + kd * (alpha - lastAlpha);
                limiarTheta = 90 + deltaLimiar;
            }
        }else
        {
            if(sentido == false)
            {
                //                cout<<"lado 1: "<<endl;
                alpha = ajustaAngulo(alpha + 180);
                v = vDelta * fabs(alpha) / limiarTheta - vMaxD;
                w = kp * alpha / 180 + kd * (alpha - lastAlpha);
                limiarTheta = 90 + deltaLimiar;
            }else if(sentido == true)
            {
                //                cout<<"lado 2: "<<endl;
                v = -vDelta * fabs(alpha) / limiarTheta + vMaxD;
                w = kp * alpha / 180 + kd * (alpha - lastAlpha);
                limiarTheta = 90 - deltaLimiar;
            }
        }

        if((fabs(robotPos.x - robotFunctions[indexRobot]->getGoal().x) < 4) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 4) )
        {
            v = 0;
            if(fabs(robotAngle) > 85 && fabs(robotAngle) < 95)
            {
                w = 0;
                //                    cout << "chegou" << endl;
            }
            else
            {
                //                    cout << "ajusta" << endl;
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = kp*alpha/180;
                }
                else
                {
                    alpha = ajustaAngulo(alpha+180);
                    w = kp*alpha/180;
                }
            }
        }
    }

    if (airball)
    {
        if( (fabs(robotPos.x - robotFunctions[indexRobot]->getGoal().x) < 4) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 4) )
        {
            v = 0;

            if(ball.pos.y > centroidDef.y)
            {
                if( robotAngle <=50 && robotAngle >= 40)
                {
                    w = 0;
                }
                else
                {
                    alpha = 45 - robotAngle;
                    alpha = ajustaAngulo(alpha);
                    if (fabs(alpha) <= limiarTheta)
                    {
                        w = 2*kp*alpha/180;
                    }
                    else
                    {
                        alpha = ajustaAngulo(alpha+180);
                        w = 2*kp*alpha/180;
                    }
                }
            }
            else
            {
                if( robotAngle >= -50 && robotAngle <= -40)
                {
                    w = 0;
                }
                else
                {
                    alpha = -45 - robotAngle;
                    alpha = ajustaAngulo(alpha);
                    if (fabs(alpha) <= limiarTheta)
                    {
                        w = 2*kp*alpha/180;
                    }
                    else
                    {
                        alpha = ajustaAngulo(alpha+180);
                        w = 2*kp*alpha/180;
                    }
                }

            }
        }
    }


    //    cout << "w: " << w << endl;
    if(w > 15)
        w = 15;
    if(w < -15)
        w = -15;
    //    cout << "Return" << endl;

    lVel = (v - w*l)*100;
    rVel = (v + w*l)*100;

    lastAlpha = alpha;
    alphaS = alpha;

    if(euclidean_dist(ball.pos,robotPos) < distGiroGol)
    {
        if(robotPos.x < ball.pos.x)
        {
            if(robotPos.y > ball.pos.y)
            {
                lVel = -velGiroGol*100;
                rVel = velGiroGol*100;
            }
            else if(robotPos.y < ball.pos.y)
            {
                lVel = velGiroGol*100;
                rVel = -velGiroGol*100;
            }
        }
        else
        {
            if(ball.pos.y > centroidDef.y + 40)
            {
                lVel = velGiroGol*100;
                rVel = -velGiroGol*100;
            }
            else if(ball.pos.y < centroidDef.y - 40)
            {
                lVel = -velGiroGol*100;
                rVel = velGiroGol*100;
            }
        }
    }


    //    cout<<"vel= "<<v<<" w="<<w << endl;
}

void Mover::striker()
{

    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float robotVel = sqrt(pow(teamRobot[indexRobot].getDataState().vel.x,2) + pow(teamRobot[indexRobot].getDataState().vel.y,2))/100;
    float robotOmega = teamRobot[indexRobot].getDataState().omega;
    float vMax = this->vMax*0.01;
    float dist = euclidean_dist(robotPos,ball.pos);

    float ct = 2;

    float teempo = (float) (clock() - clockStart)/CLOCKS_PER_SEC;

    if(tempoTroca == 0){

        if(temp == 0)
        {
            posTemp = robotPos;
            clockInvert = clock();
        }
        //cout<<"distancia: "<<euclidean_dist(robotPos,posTemp)<<endl;
        if(euclidean_dist(robotPos,posTemp) <= 1)
        {
            temp = (float) (clock() - clockInvert)/CLOCKS_PER_SEC;
            cout<<"temp"<<temp<<endl;
            if(temp >= 3){
                //   cout<<"tempoo"<<endl;
                inverte = true;
                clockTroca = clock();
                temp = 0;
            }else {
                inverte = false;
            }
        }else {
            inverte = false;
            temp = 0;
        }
    }
    if(inverte == true)
    {
        tempoTroca = (float) (clock() - clockTroca)/CLOCKS_PER_SEC;
        if(tempoTroca > 7)
            tempoTroca = 0;
    }

    if(dist < 25 && dist > 12 && robotPos.x < ball.pos.x)
    {
        vMax = vMax*dist*(0.20)/(13);//*0.3/16;
        if(vMax < 0.7)
            vMax = 0.7;
    }

    //    vMax = this->vMax;

//    if (teempo<=ct)
//    {
//        vMax=teempo*vMax/ct;
//    }


    float limiarTheta = 90;
    float deltaLimiar = 30;
    float vDelta = 0.8*vMax;
    float v, w, theta;


    // Angulo do CPU

    theta = robotFunctions[indexRobot]->getDirection();
    //PID
    if(inverte == true)
    {
        if(robotAngle > ajustaAngulo(theta) - 90 && robotAngle < ajustaAngulo(theta) + 90)
            sentido = false; //tras
        else {
            sentido = true; //frente
        }
    }
    alpha = theta - robotAngle;
    alpha = ajustaAngulo(alpha);
    alphaS = alpha;
    //        inverte = false;
    if(inverte == false)
    {
        //                    cout << "alpha: " << alpha << endl;
        if((fabs(alpha) < 10)||(fabs(alpha) > 170))
        {
            //            cout << "Primeiro caso" << endl;
            // kp = 15;
            // kd = 1.2;
        }
        else
        {
            //            cout << "Segundo caso" << endl;
        }
        if (fabs(alpha) <= limiarTheta)
        {
            v = -vDelta * fabs(alpha) / limiarTheta + vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta * fabs(alpha) / limiarTheta - vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 + deltaLimiar;
        }
    }
    else
    {
        if(sentido == false)
        {
            //            cout<<"lado 1: "<<endl;
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta * fabs(alpha) / limiarTheta - vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 + deltaLimiar;
        }else //if(sentido == true)
        {
            //            cout<<"lado 2: "<<endl;
            v = -vDelta * fabs(alpha) / limiarTheta + vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 - deltaLimiar;
        }
    }


    if(w > 21)
        w = 21;
    if(w < -21)
        w = -21;

    if(robotFunctions[indexRobot]->getStopOnGoal() == true)
    {
        theta = robotFunctions[indexRobot]->getDirection();
        alpha = theta - robotAngle;
        alpha = ajustaAngulo(alpha);
        if(euclidean_dist(robotPos,robotFunctions[indexRobot]->goal) < 25)
        {
            vMax *= 0.6;
            vDelta = vMax*0.8;
        }
        if (fabs(alpha) <= limiarTheta)
        {
            v = -vDelta * fabs(alpha) / limiarTheta + vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta * fabs(alpha) / limiarTheta - vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 + deltaLimiar;
        }

        //                cout << "Waiting for stopp" << endl;
        if((fabs(robotPos.x - robotFunctions[indexRobot]->getGoal().x) < 3) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 3) )
        {
            v = 0;
            if(fabs(robotAngle) > 85 && fabs(robotAngle) < 95)
            {
                w = 0;
                //                                    cout << "chegou" << endl;
            }
            else
            {
                //                                                    cout << "ajusta" << endl;
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = 3*kp*alpha/180;
                }
                else
                {
                    alpha = ajustaAngulo(alpha+180);
                    w = 3*kp*alpha/180;
                }
            }
        }

    }
    lVel = (v - w*l)*100;
    rVel = (v + w*l)*100;

    //  cout << "alpha: " << alpha << endl;
    //    cout << "derivada: " << alpha - lastAlpha << endl;
    //  cout << "v: " << v << endl;
    //cout << "w: " << w << endl;
//        cout << "lVel: " << lVel << endl;
//        cout << "rVel: " << rVel << endl;

    lastAlpha = alpha;
    if(robotFunctions[indexRobot]->getAgainsTheTeam() == false)
        rotate();
    else
        rotateInv();

    if(robotFunctions[indexRobot]->getAtkSituation() == true)
    {
        if(againstTheTeam == false)
        {
            if(euclidean_dist(ball.pos,robotPos) < 10)
            {
                if(firstAceleration == true)
                {
                    clockAceleration = clock();
                }
                firstAceleration = false;
                atkSituation();
            }
            else
            {
                firstAceleration = true;
            }
        }
        else
            atkSituationInv();

    }
    else
    {
        firstAceleration = true;
    }

    //    if(robotFunctions[indexRobot]->getKickState() == true)
    //    {
    //        kickPenalty();
    //    }

}

void Mover::fake9()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float robotVel = sqrt(pow(teamRobot[indexRobot].getDataState().vel.x,2) + pow(teamRobot[indexRobot].getDataState().vel.y,2))/100;

    float veMaximo = vMax*0.80*0.01;
    float v,w,theta,alpha;
    float limiarTheta = 90;
    float deltaLimiar = 30;
    float vDelta = 0.9*veMaximo;

    float teempo = (float) (clock() - clockStart)/CLOCKS_PER_SEC;

    if(tempoTroca == 0){

        if(temp == 0)
        {
            posTemp = robotPos;
            clockInvert = clock();
        }
        //cout<<"distancia: "<<euclidean_dist(robotPos,posTemp)<<endl;
        if(euclidean_dist(robotPos,posTemp) <= 1)
        {
            temp = (float) (clock() - clockInvert)/CLOCKS_PER_SEC;
            //            cout<<"temp"<<temp<<endl;
            if(temp >= 2){
                //                cout<<"tempoo"<<endl;
                inverte = true;
                clockTroca = clock();
                temp = 0;
            }else {
                inverte = false;
            }
        }else {
            inverte = false;
            temp = 0;
        }
    }
    if(inverte == true)
    {
        tempoTroca = (float) (clock() - clockTroca)/CLOCKS_PER_SEC;
        if(tempoTroca > 7)
            tempoTroca = 0;
    }


    if (euclidean_dist(robotPos, robotFunctions[indexRobot]->getGoal()) < 15)
    {
        veMaximo = 0.7;
        vDelta = veMaximo*0.9;
    }

    theta = robotFunctions[indexRobot]->getDirection();

    if(inverte == true)
    {
        if(robotAngle > theta - 90 && robotAngle < theta + 90)
            sentido = false; //tras
        else {
            sentido = true; //frente
        }
    }
    alpha = ajustaAngulo(theta) - ajustaAngulo(robotAngle);
    alpha = ajustaAngulo(alpha);
    alphaS = alpha;

    if(inverte == false)
    {
        //        cout<<"falso"<<endl;
        if (fabs(alpha) <= limiarTheta)
        {
            v = -vDelta * fabs(alpha) / limiarTheta + veMaximo;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta * fabs(alpha) / limiarTheta - veMaximo;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 + deltaLimiar;
        }
    }
    else
    {
        if(sentido == false)
        {
            //            cout<<"lado 1: "<<endl;
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta * fabs(alpha) / limiarTheta - veMaximo;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 + deltaLimiar;
        }else if(sentido == true)
        {
            //            cout<<"lado 2: "<<endl;
            v = -vDelta * fabs(alpha) / limiarTheta + veMaximo;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 - deltaLimiar;
        }
    }

    //    float angBall = angleTwoPoints(robotPos,ball.pos);
    if((fabs(robotPos.x - robotFunctions[indexRobot]->getGoal().x) < 5) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 4) )
    {
        v = 0;
        if(robotAngle > -10 && robotAngle < 10)
        {
            w = 0;
            //                                    cout << "chegou" << endl;
        }
        else
        {
            //                                    cout << "ajusta" << endl;
            alpha = 0 - robotAngle;
            alpha = ajustaAngulo(alpha);
            if (fabs(alpha) <= limiarTheta)
            {
                w = 1.7*kp*alpha/180;
                //                w = kp*alpha/180 + kd*(alpha - lastAlpha);
            }
            else
            {
                alpha = ajustaAngulo(alpha+180);
                w = 1.7*kp*alpha/180;
                //                w = kp*alpha/180 + kd*(alpha - lastAlpha);
            }
        }
    }


    lVel = 100*(v - w*l);
    rVel = 100*(v + w*l);

    lastAlpha = alpha;
    lastVel = robotVel;
}

void Mover::midfield()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    Point2f goalPos = robotFunctions[indexRobot]->getGoal();
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float robotVel = sqrt(pow(teamRobot[indexRobot].getDataState().vel.x,2) + pow(teamRobot[indexRobot].getDataState().vel.y,2))/100;
    float robotOmega = teamRobot[indexRobot].getDataState().omega;
    float vMax = 0.9;
    float dist = euclidean_dist(robotPos,ball.pos);
    int smf = 1.5;
    float smft = 0;
    float limiarTheta = 90;
    float deltaLimiar = 40;
    float vDelta = 0.8*vMax;
    float v, w, theta;
    float vPrev, vMaxPrevision=1.2, vDeltaPrev;
    Point2f prevGoal;
    if(robotFunctions[indexRobot]->getCrossing() == false)
    {
        robotFunctions[indexRobot]->fedUp = true;
        theta = robotFunctions[indexRobot]->getDirection();
        alpha = ajustaAngulo(theta) - ajustaAngulo(robotAngle);
        alpha = ajustaAngulo(alpha);


        if(euclidean_dist(robotPos,goalPos) < 15)
        {
            vMax = 0.5;
            vDelta = vMax*0.8;
        }

        if (fabs(alpha) <= limiarTheta)
        {
            v = -vDelta * fabs(alpha) / limiarTheta + vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta * fabs(alpha) / limiarTheta - vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 + deltaLimiar;
        }

        if(euclidean_dist(robotPos,goalPos) < 6)
        {
            v = 0;
            if(fabs(robotAngle) > -10 && fabs(robotAngle) < 10)
            {
                w = 0;
                //                                    cout << "chegou" << endl;
            }
            else
            {
                //                                    cout << "ajusta" << endl;
                alpha = 0 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = 2*kp*alpha/180;
                    //                    w = kp*alpha/180 + kd*(alpha - lastAlpha);
                }
                else
                {
                    alpha = ajustaAngulo(alpha+180);
                    w = 2*kp*alpha/180;
                    //                    w = kp*alpha/180 + kd*(alpha - lastAlpha);
                }
            }
                    if(ball.pos.y > centroidAtk.y)
                    {
                        if((robotAngle > -20 && robotAngle < 0)||(robotAngle > 160 && robotAngle < 180))
                        {
                            w = 0;
                            //cout << "chegou 1" << endl;
                        }
                        else
                        {
                            //cout << "ajusta" << endl;
                            alpha = (-10) - robotAngle;
                            alpha = ajustaAngulo(alpha);
                            if (fabs(alpha) <= limiarTheta)
                            {
                                w = 2*kp*alpha/180;
                            }
                            else
                            {
                                alpha = ajustaAngulo(alpha+180);
                                w = 2*kp*alpha/180;
                            }
                        }
                    }
                    else
                    {
                        if((robotAngle < 20 && robotAngle > 0)||(robotAngle < -160 && robotAngle > -180))
                        {
                            w = 0;
                            //cout << "chegou" << endl;
                        }
                        else
                        {
                            //cout << "ajusta" << endl;
                            alpha = 10 - robotAngle;
                            alpha = ajustaAngulo(alpha);
                            if (fabs(alpha) <= limiarTheta)
                            {
                                w = 2*kp*alpha/180;
                            }
                            else
                            {
                                alpha = ajustaAngulo(alpha+180);
                                w = 2*kp*alpha/180;
                            }
                        }
                    }

        }
    }
    else
    {

        smft = (float) (clock() - robotFunctions[indexRobot]->speedmf)/CLOCKS_PER_SEC;
        theta = robotFunctions[indexRobot]->getDirection();
        alpha = ajustaAngulo(theta) - ajustaAngulo(robotAngle);
        alpha = ajustaAngulo(alpha);
//        if (smft <= smf)
//        {
//            //vMax=smft*vMax/smf;
//        }
        //cout << "v:"<<vMax << endl;
        if (fabs(alpha) <= limiarTheta)
        {
            v = -vDelta * fabs(alpha) / limiarTheta + vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta * fabs(alpha) / limiarTheta - vMax;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 + deltaLimiar;
        }

    }

    lVel = 100*(v - w*l);
    rVel = 100*(v + w*l);

    lastAlpha = alpha;


    if(robotFunctions[indexRobot]->getAtkSituation() == true)
    {
        if(againstTheTeam == false)
        {
            if(euclidean_dist(ball.pos,robotPos) < 10)
            {
                if(firstAceleration == true)
                {
                    clockAceleration = clock();
                }
                firstAceleration = false;
                atkSituation();
            }
            else
            {
                firstAceleration = true;
            }
        }
        else
            atkSituationInv();

    }
    else
    {
        firstAceleration = true;
    }

}

void Mover::wing()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    Point2f goalPos = robotFunctions[indexRobot]->getGoal();
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float robotVel = sqrt(pow(teamRobot[indexRobot].getDataState().vel.x,2) + pow(teamRobot[indexRobot].getDataState().vel.y,2))/100;
    float robotOmega = teamRobot[indexRobot].getDataState().omega;
    float vMax = this->vMax*0.01;
    float dist = euclidean_dist(robotPos,ball.pos);

    float limiarTheta = 90;
    float deltaLimiar = 40;
    float vDelta = 0.8*vMax;
    float v, w, theta;
    float vTurn = 60;

    if(euclidean_dist(robotPos,goalPos) < 15 && goalPos.x != ball.pos.x && goalPos.y != ball.pos.y)
    {
        vMax = 0.6;
        vDelta = vMax*0.8;
    }

    theta = robotFunctions[indexRobot]->getDirection();
    alpha = ajustaAngulo(theta) - ajustaAngulo(robotAngle);
    alpha = ajustaAngulo(alpha);

    if (fabs(alpha) <= limiarTheta)
    {
        v = -vDelta * fabs(alpha) / limiarTheta + vMax;
        w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
        limiarTheta = 90 - deltaLimiar;
    }
    else
    {
        alpha = ajustaAngulo(alpha + 180);
        v = vDelta * fabs(alpha) / limiarTheta - vMax;
        w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
        limiarTheta = 90 + deltaLimiar;
    }

    if(robotFunctions[indexRobot]->getStopOnGoal() == true)
    {
        if(euclidean_dist(robotPos,goalPos) < 8)
        {
            v = 0;
            if((robotAngle > 80 && robotAngle < 100)||(robotAngle > -100 && robotAngle < -80))
            {
                w = 0;
                //                                    cout << "chegou" << endl;
            }
            else
            {
                //                                    cout << "ajusta" << endl;
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = 2*kp*alpha/180;
                }
                else
                {
                    alpha = ajustaAngulo(alpha+180);
                    w = 2*kp*alpha/180;
                }
            }
        }
    }

    lVel = 100*(v - w*l);
    rVel = 100*(v + w*l);

    lastAlpha = alpha;
    rotate();
    if((ball.pos.x > centroidAtk.x - 20) && euclidean_dist(robotPos, ball.pos) < 7)
    {
        if(ball.pos.y < centroidAtk.y)
        {
            lVel = -vTurn;
            rVel = vTurn;
            //            cout << "turn 1" << endl;
        }
        else if (ball.pos.y >= centroidAtk.y )
        {
            lVel = vTurn;
            rVel = -vTurn;
            //            cout << "turn 2" << endl;
        }
    }
}

void Mover::volante()
{
    Point2f meta = robotFunctions[indexRobot]->getGoal();
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;

    float vFollow = 0.4;
    float vDelta = 0.6*vMax*0.01;
    float vMaxPrevision = 1;
    float limiarTheta = 90;
    float deltaLimiar = 30;
    float v, w, theta, alpha;

    alpha = 90 - robotAngle;
    alpha = ajustaAngulo(alpha);

    if(fabs(alpha) <= limiarTheta)
    {
        w = kp*alpha/180 + kd*(alpha - lastAlpha);
        limiarTheta = 90 - deltaLimiar;
    }
    else
    {
        alpha = ajustaAngulo(alpha + 180);
        w = kp*alpha/180 + kd*(alpha - lastAlpha);
        limiarTheta = 90 + deltaLimiar;
    }

    float time = (robotPos.x + 4.5 - ball.pos.x)/ball.vel.x;
    float positionY = ball.pos.y - time*ball.vel.y;
    if(positionY < 0)
    {
        positionY = 0;
    }
    if(positionY > 130)
    {
        positionY = 130;
    }

    if((ball.vel.x < 0) && (ball.pos.x > (centroidDef.x + robotFunctions[indexRobot]->getVolanteLine())))
    {
        if(robotAngle > 0)
        {
            v = -kp*(positionY - robotPos.y)/time;
            if(v > vMaxPrevision)
            {
                v = vMaxPrevision;
            }
            else if (v < -vMaxPrevision)
            {
                v = -vMaxPrevision;
            }
            rVel = v - w*l;
            lVel = v + w*l;
        }
        else if(robotAngle < 0)
        {
            v = kp*(positionY-robotPos.y)/time;
            if(v > vMaxPrevision)
            {
                v = vMaxPrevision;
            }
            else if(v < -vMaxPrevision)
            {
                v = -vMaxPrevision;
            }
            rVel = v - w*l;
            lVel = v + w*l;
        }
    }
    else if((ball.pos.x > (centroidDef.x + robotFunctions[indexRobot]->getVolanteLine())) && (robotPos.x < (centroidDef.x + robotFunctions[indexRobot]->getVolanteLine() + 10)) && (robotPos.x > (centroidDef.x + robotFunctions[indexRobot]->getVolanteLine() - 10)))
    {
        alpha = 90 - robotAngle;
        alpha = ajustaAngulo(alpha);
        if (fabs(alpha) <= limiarTheta)
        {
            w = kp*alpha/180 + kd*(alpha - lastAlpha);
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            w = kp*alpha/180 + kd*(alpha - lastAlpha);
            limiarTheta = 90 + deltaLimiar;
        }
        if(fabs(robotPos.y - ball.pos.y) < 3)
        {
            v = 0;
        }
        else if((robotAngle > 0) && (ball.pos.y < robotPos.y))
        {
            v = vFollow;
        }
        else if((robotAngle > 0) && (ball.pos.y > robotPos.y))
        {
            v = -vFollow;
        }
        else if((robotAngle < 0) && (ball.pos.y < robotPos.y))
        {
            v = -vFollow;
        }
        else if((robotAngle < 0) && (ball.pos.y > robotPos.y))
        {
            v = vFollow;
        }
    }
    else
    {
        theta = angleTwoPoints(robotPos, meta);
        alpha = theta - robotAngle;
        alpha = ajustaAngulo(alpha);

        if(fabs(alpha) <= limiarTheta)
        {
            v = -vDelta*fabs(alpha)/limiarTheta + 0.4;
            w = kp*alpha/180 + kd*(alpha - lastAlpha);
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta*fabs(alpha)/limiarTheta - 0.4;
            w = kp*alpha/180 + kd*(alpha - lastAlpha);
            limiarTheta = 90 + deltaLimiar;
        }

        if ((fabs(alpha) > 65) && (fabs(alpha) < 115))
        {
            v = 0;
        }

        if((fabs(robotPos.x - robotFunctions[indexRobot]->getGoal().x) < 4) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 5) &&(fabs(robotAngle) > 85) && (fabs(robotAngle) < 95))
        {
            v = 0;
            w = 0;
        }
        else if ((fabs(robotPos.x - robotFunctions[indexRobot]->getGoal().x) < 4) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 5))
        {
            alpha = 90 - robotAngle;
            alpha = ajustaAngulo(alpha);
            if (fabs(alpha) <= limiarTheta)
            {
                w = kp*alpha/180 + kd*(alpha - lastAlpha);
            }
            else
            {
                alpha = ajustaAngulo(alpha+180);
                w = kp*alpha/180 + kd*(alpha - lastAlpha);
            }
            v = 0;
        }
    }
    if(w > 15)
    {
        w = 15;
    }
    if(w < -15)
    {
        w = -15;
    }

    lVel = v - w*l;
    rVel = v + w*l;
    lastAlpha = alpha;

    rotate();
    kickRotate();
}


void Mover::libero()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float robotVel = sqrt(pow(teamRobot[indexRobot].getDataState().vel.x,2) + pow(teamRobot[indexRobot].getDataState().vel.y,2))/100;

    float veMaximo = vMax*0.80*0.01;
    float v,w,theta,alpha;
    float limiarTheta = 90;
    float deltaLimiar = 30;
    float vDelta = 0.9*veMaximo;



    if (euclidean_dist(robotPos, robotFunctions[indexRobot]->getGoal()) < 15)
    {
        veMaximo = 0.7;
        vDelta = veMaximo*0.9;
    }

    theta = robotFunctions[indexRobot]->getDirection();


    alpha = ajustaAngulo(theta) - ajustaAngulo(robotAngle);
    alpha = ajustaAngulo(alpha);
    alphaS = alpha;

    if (fabs(alpha) <= limiarTheta)
    {
        v = -vDelta * fabs(alpha) / limiarTheta + veMaximo;
        w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
        limiarTheta = 90 - deltaLimiar;
    }
    else
    {
        alpha = ajustaAngulo(alpha + 180);
        v = vDelta * fabs(alpha) / limiarTheta - veMaximo;
        w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
        limiarTheta = 90 + deltaLimiar;
    }

    lVel = 100*(v - w*l);
    rVel = 100*(v + w*l);

    lastAlpha = alpha;
    lastVel = robotVel;

    int distGiro = 7;

    if( ball.pos.x <= 85)
    {
        if((ball.pos.y >= centroidAtk.y) && (euclidean_dist(ball.pos,robotPos) < distGiro))
        {
            lVel = -1*100;
            rVel = 100;
        }
        else if((ball.pos.y < centroidAtk.y) && (euclidean_dist(ball.pos,robotPos) < distGiro))
        {
            lVel = 100;
            rVel = -1*100;
        }
    }
    else
    {
        if((ball.pos.y < centroidAtk.y) && (euclidean_dist(ball.pos,robotPos) < distGiro))
        {
            lVel = -1*100;
            rVel = 100;
        }
        else if((ball.pos.y >= centroidAtk.y) && (euclidean_dist(ball.pos,robotPos) < distGiro))
        {
            lVel = 100;
            rVel = -1*100;
        }
    }
}

void Mover::offdefender()
{
    float vMaxD = 0.65;
    float vFollow = 0.50;
    float vMaxPrevision = 0.85;
    float vDelta = 1*vMaxD;
    float limiarTheta = 90;
    float deltaLimiar = 30;
    float vPrev, vDeltaPrev;
    float distGiroGol = 10;
    float velGiroGol = 1.0;
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float robotVel = sqrt(pow(teamRobot[indexRobot].getDataState().vel.x,2) + pow(teamRobot[indexRobot].getDataState().vel.y,2))/100;

    float v,w,theta;
    float time = (robotPos.x+5 - ball.pos.x)/ball.vel.x;
    float positionY = ball.pos.y + time*ball.vel.y;

    if(positionY < 0)
    {
        positionY = 0;
    }
    if(positionY > 130)
    {
        positionY = 130;
    }

    if (ball.vel.x < -20 && ball.pos.x > centroidDef.x + offLine)
    {
        vPrev = fabs(0.01 * (positionY - robotPos.y) / time);

        if (vPrev > vMaxPrevision)
        {
            vPrev = vMaxPrevision;
        }
        else if (vPrev < 0.4)
        {
            vPrev = 0.4;
        }
        vDeltaPrev = vPrev * 1;

        if(robotPos.x < centroidDef.x + offLine + 5 && robotPos.x > centroidDef.x + offLine - 5)
        {
            prevGoal = Point2f(robotPos.x, positionY);
        }
        else
        {            prevGoal = Point2f(centroidDef.x + offLine, positionY);
        }

        theta = angleTwoPoints(robotPos, prevGoal);
        alpha = theta - robotAngle;
        alpha = ajustaAngulo(alpha);
        if (fabs(alpha) <= limiarTheta)
        {
            v = -vDeltaPrev * fabs(alpha) / limiarTheta + vPrev;
            w = kp * alpha / 180 + kd * (alpha - lastAlpha);
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha + 180);
            v = vDeltaPrev * fabs(alpha) / limiarTheta - vPrev;
            w = kp * alpha / 180 + kd * (alpha - lastAlpha);
            limiarTheta = 90 + deltaLimiar;
        }

        if ((fabs(robotPos.x - prevGoal.x) < 2) && (fabs(robotPos.y - prevGoal.y) < 2))
        {
            v = 0;
            if (fabs(robotAngle) > 85 && fabs(robotAngle) < 95)
            {
                w = 0;
            }
            else
            {
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = kp * alpha / 180;
                }
                else
                {
                    alpha = ajustaAngulo(alpha + 180);
                    w = kp * alpha / 180;
                }
            }
        }

                  cout << "PrevY: " << positionY << endl;
        //          cout << "ballY: " << ball.pos.y << endl;
        //                  cout << "Prevision vel: " << v << endl;
    }
    else if (ball.pos.x > centroidDef.x + offLine && robotPos.x < centroidDef.x + offLine + 7 && robotPos.x > centroidDef.x + offLine - 7)
    {
        alpha = 90 - robotAngle;
        alpha = ajustaAngulo(alpha);
        if (fabs(alpha) <= limiarTheta)
        {
            w = kp*alpha/180 + kd*(alpha - lastAlpha);
            limiarTheta = 90 - deltaLimiar;
        }
        else
        {
            alpha = ajustaAngulo(alpha+180);
            w = kp*alpha/180 + kd*(alpha - lastAlpha);
            limiarTheta = 90 + deltaLimiar;
        }

        if(fabs(robotPos.y-ball.pos.y) < 3)
        {
            v = 0;
        }
        else if (robotAngle > 0 && ball.pos.y < robotPos.y)
        {
            v = -vFollow;
        }
        else if (robotAngle > 0 && ball.pos.y > robotPos.y)
        {
            v = vFollow;
        }
        else if (robotAngle < 0 && ball.pos.y < robotPos.y)
        {
            v = vFollow;
        }
        else if (robotAngle < 0 && ball.pos.y > robotPos.y)
        {
            v = -vFollow;
        }
                cout << "Follow" << endl;
    }
    else
    {
            cout << "Return" << endl;
        if (euclidean_dist(robotPos, robotFunctions[indexRobot]->getGoal()) < 10)
        {
            vMaxD = 0.4;
        }
        else
        {
            vMaxD = vMaxD;
        }
        vDelta = vMaxD*1;
        theta = robotFunctions[indexRobot]->getDirection();

        alpha = theta - robotAngle;
        alpha = ajustaAngulo(alpha);

            if (fabs(alpha) <= limiarTheta)
            {
                v = -vDelta * fabs(alpha) / limiarTheta + vMaxD;
                w = kp * alpha / 180 + kd * (alpha - lastAlpha);
                limiarTheta = 90 - deltaLimiar;
            }
            else
            {
                alpha = ajustaAngulo(alpha + 180);
                v = vDelta * fabs(alpha) / limiarTheta - vMaxD;
                w = kp * alpha / 180 + kd * (alpha - lastAlpha);
                limiarTheta = 90 + deltaLimiar;
            }


        if((fabs(robotPos.x - robotFunctions[indexRobot]->getGoal().x) < 4) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 4) )
        {
            v = 0;
            if(fabs(robotAngle) > 85 && fabs(robotAngle) < 95)
            {
                w = 0;
                //                    cout << "chegou" << endl;
            }
            else
            {
                //                    cout << "ajusta" << endl;
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = kp*alpha/180;
                }
                else
                {
                    alpha = ajustaAngulo(alpha+180);
                    w = kp*alpha/180;
                }
            }
        }
    }

    //    cout << "w: " << w << endl;
    if(w > 15)
        w = 15;
    if(w < -15)
        w = -15;
    //    cout << "Return" << endl;

    lVel = (v - w*l)*100;
    rVel = (v + w*l)*100;

    lastAlpha = alpha;
    alphaS = alpha;

    if(euclidean_dist(ball.pos,robotPos) < distGiroGol)
    {
        if(robotPos.x < ball.pos.x)
        {
            if(robotPos.y > ball.pos.y)
            {
                lVel = -velGiroGol*100;
                rVel = velGiroGol*100;
            }
            else if(robotPos.y < ball.pos.y)
            {
                lVel = velGiroGol*100;
                rVel = -velGiroGol*100;
            }
        }
        else
        {
            if(ball.pos.y > centroidDef.y + 40)
            {
                lVel = velGiroGol*100;
                rVel = -velGiroGol*100;
            }
            else if(ball.pos.y < centroidDef.y - 40)
            {
                lVel = -velGiroGol*100;
                rVel = velGiroGol*100;
            }
        }
    }

}


void Mover::setGains(MatrixXd mat)
{
    robotGains = mat;
}

MatrixXd Mover::getGains()
{
    return robotGains;
}

void Mover::rotate()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;

    // Função para fazer o robô girar nos cantos
    if ((ball.pos.y > (centroidAtk.y + 60)) && (euclidean_dist(ball.pos, robotPos) < distGiro))
    {
        rVel = -velGiroLado*vMax;
        lVel = velGiroLado*vMax;
    }
    else if ((ball.pos.y < (centroidAtk.y - 60)) && (euclidean_dist(ball.pos,robotPos) < distGiro))
    {
        rVel = velGiroLado*vMax;
        lVel = -velGiroLado*vMax;
    }

    // Função para fazer o robô girar na linha de fundo de ataque
    if ((ball.pos.y > (centroidAtk.y + 40)) && ((euclidean_dist(ball.pos, robotPos)) < distGiro) && (fabs(ball.pos.x - centroidAtk.x) < 15))
    {
        rVel = -velGiroLado*vMax;
        lVel = velGiroLado*vMax;
    }
    else if ((ball.pos.y < (centroidAtk.y - 40)) && (euclidean_dist(ball.pos, robotPos) < distGiro) && (fabs(ball.pos.x - centroidAtk.x) < 15))
    {
        rVel = velGiroLado*vMax;
        lVel = -velGiroLado*vMax;
    }

    // Função para fazer o robô girar na linha de fundo de defesa
    if ((ball.pos.y > (centroidDef.y + 35)) && ((euclidean_dist(ball.pos, robotPos)) < distGiro) && (fabs(ball.pos.x - centroidDef.x) < 15))
    {
        rVel = -velGiroLado*vMax;
        lVel = velGiroLado*vMax;
    }
    else if ((ball.pos.y < (centroidDef.y - 35)) && (euclidean_dist(ball.pos, robotPos) < (distGiro && fabs(ball.pos.x - centroidDef.x) < 15)))
    {
        rVel = velGiroLado*vMax;
        lVel = -velGiroLado*vMax;
    }
}

void Mover::rotateInv()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    // Função para fazer o robô girar nos cantos
    if ((ball.pos.y > (centroidAtk.y + 60)) && (euclidean_dist(ball.pos, robotPos) < distGiro))
    {
        rVel = velGiroLado*vMax/100;
        lVel = -velGiroLado*vMax/100;
    }
    else if ((ball.pos.y < (centroidAtk.y - 60)) && (euclidean_dist(ball.pos,robotPos) < distGiro))
    {
        rVel = -velGiroLado*vMax/100;
        lVel = velGiroLado*vMax/100;
    }

    // Função para fazer o robô girar na linha de fundo de ataque
    if ((ball.pos.y > (centroidAtk.y + 40)) && ((euclidean_dist(ball.pos, robotPos)) < distGiro) && (fabs(ball.pos.x - centroidAtk.x) < 15))
    {
        rVel = velGiroLado*vMax/100;
        lVel = -velGiroLado*vMax/100;
    }
    else if ((ball.pos.y < (centroidAtk.y - 40)) && (euclidean_dist(ball.pos, robotPos) < distGiro) && (fabs(ball.pos.x - centroidAtk.x) < 15))
    {
        rVel = -velGiroLado*vMax/100;
        lVel = velGiroLado*vMax/100;
    }

    // Função para fazer o robô girar na linha de fundo de defesa
    if ((ball.pos.y > (centroidDef.y + 35)) && ((euclidean_dist(ball.pos, robotPos)) < distGiro) && (fabs(ball.pos.x - centroidDef.x) < 15))
    {
        rVel = velGiroLado*vMax/100;
        lVel = -velGiroLado*vMax/100;
    }
    else if ((ball.pos.y < (centroidDef.y - 35)) && (euclidean_dist(ball.pos, robotPos) < (distGiro && fabs(ball.pos.x - centroidDef.x) < 15)))
    {
        rVel = -velGiroLado*vMax/100;
        lVel = velGiroLado*vMax/100;
    }
}

void Mover::kickRotate()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;

    if ((ball.pos.y > robotPos.y) && (euclidean_dist(ball.pos, robotPos) < distGiro))
    {
        rVel = -velGiroLado;
        lVel = velGiroLado;
    }
    else if ((ball.pos.y < robotPos.y) && (euclidean_dist(ball.pos, robotPos) < distGiro))
    {
        rVel = velGiroLado;
        lVel = -velGiroLado;
    }
}

void Mover::atkSituation()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    Point2f eixoX(1.0,0.0);
    float vAtk = 120*0.01;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float angBallRobot = robotAngle-angleTwoPoints(robotPos,ball.pos);
    angBallRobot = ajustaAngulo(angBallRobot);
    float ct = 1.5;
    float teempo = 0;
    float vTurn = 120*0.01;

    if(firstAceleration == false)
    {
        teempo = (float) (clock() - clockAceleration)/CLOCKS_PER_SEC;
        if (teempo<=ct)
        {
            vAtk=teempo*vAtk/ct;
        }
        if(robotFunctions[indexRobot]->getAtkSituationTiro() == true)
        {
            //            cout << "ahhhhh" << endl;
            if(vAtk < 1)
                vAtk = 1;
        }
        else
        {
            //            cout << "bbbbbb" << endl;
            if(vMax < 80)
            {
                if(vAtk < 0.7)
                    vAtk = 0.7;
            }
            else
            {
                if(vAtk < vMax*0.01*0.8)
                    vAtk = vMax*0.01*0.8;
            }
        }
        //        cout << "vAtk: " << vAtk << endl;
    }

    if (euclidean_dist(ball.pos,robotPos) < 10){
        if (fabs(robotAngle) < 90 &&  fabs(angBallRobot) < 30){
            lVel = vAtk*100;
            rVel = vAtk*100;
        }
        if(fabs(angBallRobot) > 150){
            lVel = -vAtk*100;
            rVel = -vAtk*100;
        }
        if(euclidean_dist(robotPos, centroidAtk)<25 && robotFunctions[indexRobot]->getlittleChute()==true)
        {
            if(robotPos.y > centroidAtk.y)
            {
                lVel = vTurn*100;
                rVel = -vTurn*100;
            }
            else
            {
                lVel = -vTurn*100;
                rVel = vTurn*100;
            }
            //            cout<<"endf"<<endl;

        }
        //cout << "Ataque Situation any point" << endl;
    }
}

void Mover::atkSituationInv()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    Point2f eixoX(1.0,0.0);
    float vAtk = 100*0.01;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float angBallRobot = robotAngle-angleTwoPoints(robotPos,ball.pos);
    angBallRobot = ajustaAngulo(angBallRobot);

    if (euclidean_dist(ball.pos,robotPos) < 13){
        if (fabs(robotAngle) < 90 &&  fabs(angBallRobot) < 30){
            lVel = -vAtk;
            rVel = -vAtk;
        }
        if(fabs(angBallRobot) > 150){
            lVel = vAtk;
            rVel = vAtk;
        }
        //cout << "Ataque Situation any point" << endl;
    }
}

void Mover::setDirection(bool d)
{
    robotDirection = d;
}

void Mover::kickPenalty()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;

    if ((ball.pos.y > robotPos.y) && (euclidean_dist(ball.pos, robotPos) < 14) && euclidean_dist(robotPos,Point2f(122,65)) < 10)
    {
        rVel = -velGiroPenalty;
        lVel = velGiroPenalty;
        cout << "Chute Esquerdo" << endl;
    }
    else if ((ball.pos.y < robotPos.y) && (euclidean_dist(ball.pos, robotPos) < distGiro) && euclidean_dist(robotPos,Point2f(122,65)) < 10)
    {
        rVel = velGiroPenalty;
        lVel = -velGiroPenalty;
        cout << "Chute Direito" << endl;
    }
    else{
        cout << "Sem Chute" << endl;
    }
}

void Mover::setStrategy(int s)
{
    strategy = s;
}

int Mover::getStrategy()
{
    return strategy;
}


//float Mover::twiddle ()
//{
//    float deltinha = 0.001; //Mínimo valor de delta_k
//    float epsilon = 0.1; //Valor de incremento de delta_k
//    float kp = 0;
//    float kd = 0;
//    float ki = 0;
//    float k[3] = {kp, ki, kd}; //Vetor de ganhos
//    float delta_k[3] = {5, 5, 5}; //Vetor de incremento do vetor de ganho
//    float Max_err = 1000; //Máximo erro aceitável
//    float delta_Er_best = 0; //Iniciando variável de melhor erro
//    float i = 1;
//    int j;
//    float alpha, a, b, c, t, v, w, theta, r0;
//    float limiarTheta = 90;
//    float deltaLimiar = 30;
//    float vMax = this->vMax;
//    float vDelta = 0.6*vMax*0.01;
//    float limitr0 = 0.01;  //Erro de posição para parada

//    //Posição inicial do robô
//    Point2d robotPos=teamRobot[indexRobot].getDataState().pos;
//    robotPos.x = 20;
//    robotPos.y = 20;
//    float robotAngle = teamRobot[indexRobot].getDataState().angle;
//    robotAngle = 0;

//    //Posição da meta
//    Point2d meta = robotFunctions[indexRobot]->getGoal();
//    meta.x = 120;
//    meta.y = 100;
//    alpha = 90 - robotAngle;
//    alpha = ajustaAngulo(alpha);

//    float dt = 0.01; //Variável de tempo

//    //Reta entre 2 pontos
//    a = robotPos.y - meta.y;
//    b = robotPos.x - meta.x;
//    c = robotPos.x*meta.y - meta.y*robotPos.x;

//    if (meta.x>0)
//    {
//        t=robotPos.x;
//        while (t<meta.x)
//            t++;
//    }
//    else
//    {
//        t=meta.x;
//        while (t<robotPos.x)
//            t++;
//    }

//    //Distancia do robô e da meta
//    r0 = sqrt(pow(meta.x-robotPos.x,2)+pow(meta.y-robotPos.y,2));

//    //PID
//    v = -vDelta*fabs(alpha)/limiarTheta + 0.4;
//    w = kp*alpha/180 + kd*(alpha - lastAlpha);
//    limiarTheta = 90 - deltaLimiar;

//    //Iniciando Twiddle
//    while (i<0) //Ajustar o parâmetro do while: sum(delta_k(i.:)) > deltinha()
//    {
//        for(j = 0; j<3; j++)
//        {
//            k[j] = k[j] + delta_k[j]; //Incremento do vetor ganho

//            //Trajetória com o novo vetor de ganho
//            v = -vDelta*fabs(alpha)/limiarTheta + 0.4;
//            w = kp*alpha/180 + kd*(alpha - lastAlpha);
//            limiarTheta = 90 - deltaLimiar;

//        }
//    }


//}
void Mover:: kickGoalKeeper()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    Point2f eixoX(1.0,0.0);
    float vAtk = 100*0.01;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float angBallRobot = robotAngle-angleTwoPoints(robotPos,ball.pos);
    angBallRobot = ajustaAngulo(angBallRobot);
    float ct = 1.75;
    float teempo = 0;

    if(firstAceleration == false)
    {
        teempo = (float) (clock() - clockAceleration)/CLOCKS_PER_SEC;
        if (teempo<=ct)
        {
            vAtk=teempo*vAtk/ct;
        }
        if(vAtk < 0.5)
            vAtk = 0.5;
        cout << "vAtk: " << vAtk << endl;
    }

    if (euclidean_dist(ball.pos,robotPos) < 10){
        if (fabs(robotAngle) < 90 &&  fabs(angBallRobot) < 30){
            lVel = vAtk*100;
            rVel = vAtk*100;
        }
        if(fabs(angBallRobot) > 150){
            lVel = -vAtk*100;
            rVel = -vAtk*100;
        }
        //cout << "Ataque Situation any point" << endl;
    }
}

void Mover::setIndex(int index)
{
    indexRobot = index;
}
