#include "mover.h"

Mover::Mover()
{
    kp = 19;
    kd = 2.5;
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
            //Control();
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
        case LIBERO:
            libero();
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
    float vMaxGol = 0.85;
    kd = 0;
    kp = 10;

    float vDeltaGol = vMaxGol;
    float vPrev, vDeltaPrev;

    float v, w, theta;

    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float time;
    if(ball.vel.x < -5 )
    {
        time = (robotPos.x+5 - ball.pos.x)/ball.vel.x;
    }
    else
    {
        time = 0;
    }

    float positionY = ball.pos.y + time*ball.vel.y;

    if(positionY < 45)
    {
        positionY = 45;
    }
    else if(positionY > 85)
    {
        positionY = 85;
    }

    prevGoal = robotPos;
    if (ball.vel.x < -10 && (positionY > centroidDef.y-20) && (positionY < centroidDef.y+20))
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
        //vPrev = vMax;
        vDeltaPrev = vPrev * 1;

        if(robotPos.x < centroidDef.x + 4 && robotPos.x > centroidDef.x - 5)
        {
            prevGoal = Point2f(robotPos.x, positionY);
        }
        else
        {
            prevGoal = Point2f(centroidDef.x + 4, positionY);
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
            if (fabs(robotAngle) > 75 && fabs(robotAngle) < 105)
            {
                w = 0;
            }
            else
            {
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = kp*alpha/180 + kd*(alpha - lastAlpha);
                }
                else
                {
                    alpha = ajustaAngulo(alpha + 180);
                    w = kp*alpha/180 + kd*(alpha - lastAlpha);
                }
            }
        }
    }
    else
    {
        if ((ball.pos.x < centroidDef.x + 70) && (ball.pos.y > centroidDef.y - 25) && (ball.pos.y < centroidDef.y + 25) && (robotPos.x < centroidDef.x+5) && (robotPos.x >= centroidDef.x -5))
        {
            //cout<<"follow"<<endl;
            theta = robotFunctions[indexRobot]->getDirection();
            alpha = theta - robotAngle;
            alpha = ajustaAngulo(alpha);
            if(fabs(alpha) > 5)
            {
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
            }
            else
                w = 0;

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
        }
        else
        {
            theta = robotFunctions[indexRobot]->getDirection();
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

            if ((fabs(robotPos.x - robotFunctions[indexRobot] ->getGoal().x <4)) && (((robotFunctions[indexRobot]->getGoal().y - robotPos.y<1) && (ball.pos.y>65)) || ((robotPos.y - robotFunctions[indexRobot]->getGoal().y <1) && (ball.pos.y<65))))
            {
                v = 0;
                //cout<<"pato"<<endl;
                if(fabs(robotAngle) > 75 && fabs(robotAngle) < 105)
                {
                    //cout<<"Ta aqui"<<endl;
                    w = 0;
                }
                else
                {
                    //cout<<"Ajustando"<<endl;
                    alpha = 90 - robotAngle;
                    alpha = ajustaAngulo(alpha);
                    if (fabs(alpha) < limiarTheta)
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
    }

    if(w > 30)
        w = 30;
    if(w < -30)
        w = -30;

    lVel = (v - w*l)*100;
    rVel = (v + w*l)*100;

    lastAlpha = alpha;

    rotate_def();

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
    float distGiroGol = 7;
    float velGiroGol = 1.0;

    airball = false;

    if(ball.pos.x < centroidDef.x +15 && ball.pos.y < centroidDef.y + 40 && ball.pos.y > centroidDef.y - 40 && againstTheTeam == false ){
        airball = true;
    }

    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float v,w,theta;

    if(robotPos.x < centroidDef.x + 15 && robotPos.y < 85 && robotPos.y > 45 ){
        vMaxD = 0.8;
        vFollow = 0.7;
        vDelta = vMaxD;
    }

    if(tempoTroca == 0){

        if(temp == 0)
        {
            posTemp = robotPos;
            clockInvert = clock();
        }

        if(euclidean_dist(robotPos,posTemp) <= 0.2)
        {
            temp = (float) (clock() - clockInvert)/CLOCKS_PER_SEC;

            if(temp >= 3){
                inverte = true;
                inverte = false;
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
    inverte = false;
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
                    w = kp * alpha / 180;
                }
                else
                {
                    alpha = ajustaAngulo(alpha + 180);
                    w = kp * alpha / 180;
                }
            }
        }

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
    }
    else
    {
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
        }
        else
        {
            if(sentido == false)
            {
                alpha = ajustaAngulo(alpha + 180);
                v = vDelta * fabs(alpha) / limiarTheta - vMaxD;
                w = kp * alpha / 180 + kd * (alpha - lastAlpha);
                limiarTheta = 90 + deltaLimiar;
            }else if(sentido == true)
            {
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
            }
            else
            {
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
                        w = kp*alpha/180;
                    }
                    else
                    {
                        alpha = ajustaAngulo(alpha+180);
                        w = kp*alpha/180;
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
    }

    if(w > 60)
        w = 60;
    if(w < -60)
        w = -60;

    lVel = (v - w*l)*100;
    rVel = (v + w*l)*100;

    //    cout<< "lvel: " <<lVel<<endl;
    //    cout<< "rvel: " <<rVel<<endl;

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

void Mover::Control()
{

    float kpc = 10;
    float kdc = 2;
    float kic = 3;
    dataState robot = teamRobot[indexRobot].getDataState();
    float angleRobot = robot.angle;
    float vMax = 50*0.01;
    float v,w;
    float wMax = 0.8*vMax/l; //dessa forma o robo jamais ira rodar com as duas rodas em sentidos opostos, evitando derrapagem bruscas.
    float intAlphaMax = 200;


    float theta = robotFunctions[indexRobot]->getDirection();
    alpha = theta - angleRobot;
    alpha = ajustaAngulo(alpha);

    v = 1;
    if (fabs(alpha) > 90){
        angleRobot = ajustaAngulo(angleRobot + 180);
        alpha = ajustaAngulo(theta - angleRobot);
        v = -1;
    }

    v = ((-0*vMax * fabs(alpha) / 90) + vMax)*v;

    intAlpha = intAlpha + alpha;
    intAlpha = fmin(intAlphaMax,fmax(intAlpha, -intAlphaMax));

    w = (kpc/100 * alpha) + (kic/100000 * intAlpha) + (kdc * (alpha - lastAlpha));
    w = fmin(wMax, fmax(w , -wMax));

    lVel = (v - w*l)*100;
    rVel = (v + w*l)*100;

    lastAlpha = alpha;

    rotate();


    if(robotFunctions[indexRobot]->getAtkSituation() == true)
    {
        if(euclidean_dist(ball.pos,robot.pos) < 10)
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
    {
        firstAceleration = true;
    }

}

void Mover::striker()
{

    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float vMax = this->vMax*0.01;
    float dist = euclidean_dist(robotPos,ball.pos);

    float ct = 2;

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
            // cout<<"temp"<<temp<<endl;
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

    inverte = false;
    if(inverte == false)
    {
        //                    cout << "alpha: " << alpha << endl;
        if((fabs(alpha) < 10)||(fabs(alpha) > 170))
        {
            //cout << "Reto" << endl;
            //             kp = 20;
            //             kd = 4;
        }
        else
        {
            //cout << "Curva" << endl;
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

    float veMaximo = vMax*0.80*0.01;
    float v,w,theta,alpha;
    float limiarTheta = 90;
    float deltaLimiar = 30;
    float vDelta = 0.9*veMaximo;

    if(tempoTroca == 0){

        if(temp == 0)
        {
            posTemp = robotPos;
            clockInvert = clock();
        }
        if(euclidean_dist(robotPos,posTemp) <= 1)
        {
            temp = (float) (clock() - clockInvert)/CLOCKS_PER_SEC;
            if(temp >= 2){
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

    //precisa diminuir velocidade no simulador????
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

    if(inverte == false)
    {
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
            alpha = ajustaAngulo(alpha + 180);
            v = vDelta * fabs(alpha) / limiarTheta - veMaximo;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 + deltaLimiar;
        }else if(sentido == true)
        {
            v = -vDelta * fabs(alpha) / limiarTheta + veMaximo;
            w = (kp * alpha / 180) + (kd * (alpha - lastAlpha));
            limiarTheta = 90 - deltaLimiar;
        }
    }

    if((fabs(robotPos.x - robotFunctions[indexRobot]->getGoal().x) < 5) && (fabs(robotPos.y - robotFunctions[indexRobot]->getGoal().y) < 4) )
    {
        v = 0;
        if(robotAngle > -10 && robotAngle < 10)
        {
            w = 0;
        }
        else
        {
            alpha = 0 - robotAngle;
            alpha = ajustaAngulo(alpha);
            if (fabs(alpha) <= limiarTheta)
            {
                w = 1.7*kp*alpha/180;
            }
            else
            {
                alpha = ajustaAngulo(alpha+180);
                w = 1.7*kp*alpha/180;
            }
        }
    }


    lVel = 100*(v - w*l);
    rVel = 100*(v + w*l);

    lastAlpha = alpha;
}


void Mover::midfield()
{
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    Point2f goalPos = robotFunctions[indexRobot]->getGoal();
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float vMax = this->vMax*0.01;
    float smf = 1.5;
    float smft = 0;
    float limiarTheta = 90;
    float deltaLimiar = 40;
    float vDelta = 0.8*vMax;
    float v, w, theta;
    Point2f prevGoal;

    float time;
    Point2f ballPrev;
    if(ball.vel.y == 0)
    {
        time = 0;
        ballPrev = ball.pos;
    }
    else
    {
        time = fabs((robotPos.y - ball.pos.y)/ball.vel.y);
        ballPrev = Point2f((ball.pos.x - 2.5) + time * ball.vel.x , robotPos.y);
    }


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
            }
            else
            {
                alpha = 0 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) <= limiarTheta)
                {
                    w = kp*alpha/180;
                    //                    w = kp*alpha/180 + kd*(alpha - lastAlpha);
                }
                else
                {
                    alpha = ajustaAngulo(alpha+180);
                    w = kp*alpha/180;
                    //                    w = kp*alpha/180 + kd*(alpha - lastAlpha);
                }
            }
            if(ball.pos.y > centroidAtk.y)
            {
                if((robotAngle > -10 && robotAngle < 0)||(robotAngle > 170 && robotAngle < 180))
                {
                    w = 0;
                }
                else
                {
                    alpha =  - robotAngle;
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
            else
            {
                if((robotAngle < 10 && robotAngle > 0)||(robotAngle < -170 && robotAngle > -180))
                {
                    w = 0;
                }
                else
                {
                    alpha = - robotAngle;
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
    }
    else
    {

        smft = (float) (clock() - robotFunctions[indexRobot]->speedmf)/CLOCKS_PER_SEC;
        theta = robotFunctions[indexRobot]->getDirection();
        alpha = ajustaAngulo(theta) - ajustaAngulo(robotAngle);
        alpha = ajustaAngulo(alpha);
        if (smft <= smf)
        {
            //vMax=smft*vMax/smf;

        }
        Point2f robotPosAux = Point2f (robotPos.x + 4, robotPos.y);
        float distancia = euclidean_dist(robotPosAux, ballPrev);
        if (time == 0)
        {
            vMax = this->vMax*0.01;
        }
        else
        {
            vMax= fabs(distancia/time)*0.01;
        }
        if(vMax > 1.5)
        {
            vMax = 1.5;
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
                cout<< "Atk Situation"<<endl;
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
    float vMax = this->vMax*0.01;

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
            }
            else
            {
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
        }
        else if (ball.pos.y >= centroidAtk.y )
        {
            lVel = vTurn;
            rVel = -vTurn;
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
    }

    theta = robotFunctions[indexRobot]->getDirection();

    alpha = ajustaAngulo(theta) - ajustaAngulo(robotAngle);
    alpha = ajustaAngulo(alpha);

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

    int distGiroGol = 8;
    if(ball.pos.x < centroidDef.x + 15)
        distGiroGol = 8;

    // Quando a bola está no campo de defesa, o líbero jogará a bola para os cantos do campo, mas quando estiver no campo de ataque o mesmo a jogará para o centro do campo
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

void Mover::offdefender()
{
    float vMaxD = 0.65;
    float vFollow = 0.50;
    float vMaxPrevision = 0.85;
    float vDelta = vMaxD;
    float limiarTheta = 90;
    float deltaLimiar = 30;
    float vPrev, vDeltaPrev;
    float distGiroGol = 10;
    float velGiroGol = 1.0;
    Point2f robotPos = teamRobot[indexRobot].getDataState().pos;
    float robotAngle = teamRobot[indexRobot].getDataState().angle;
    float v,w,theta;
    float time = (robotPos.x+5 - ball.pos.x)/ball.vel.x;
    float positionY = ball.pos.y + time*ball.vel.y;
    if(time == 0)
        time = 0.1;
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

        vDeltaPrev = vPrev;

        if(robotPos.x < centroidDef.x + offLine + 5 && robotPos.x > centroidDef.x + offLine - 5)
            prevGoal = Point2f(robotPos.x, positionY);
        else
            prevGoal = Point2f(centroidDef.x + offLine, positionY);

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
                if (fabs(alpha) >= limiarTheta)
                    alpha = ajustaAngulo(alpha + 180);
                w = kp * alpha / 180;
            }
        }
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

        if(fabs(robotPos.y - ball.pos.y) < 7)
        {
            v = 0;
        }
        else if ((robotAngle > 0 && ball.pos.y < robotPos.y) || (robotAngle < 0 && ball.pos.y > robotPos.y))
        {
            v = -vFollow;
        }
        else if ((robotAngle > 0 && ball.pos.y > robotPos.y) || (robotAngle < 0 && ball.pos.y < robotPos.y))
        {
            v = vFollow;
        }
    }
    else
    {
        if (euclidean_dist(robotPos, robotFunctions[indexRobot]->getGoal()) < 10)
        {
            vMaxD = 0.4;
        }

        vDelta = vMaxD;
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
                w = 0;
            else
            {
                alpha = 90 - robotAngle;
                alpha = ajustaAngulo(alpha);
                if (fabs(alpha) >= limiarTheta)
                    alpha = ajustaAngulo(alpha + 180);
                w = kp * alpha / 180;
            }
        }
    }

    if(w > 15)
        w = 15;
    if(w < -15)
        w = -15;

    lVel = (v - w*l)*100;
    rVel = (v + w*l)*100;

    lastAlpha = alpha;

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
        rVel = velGiroLado*vMax;
        lVel = -velGiroLado*vMax;
    }
    else if ((ball.pos.y < (centroidAtk.y - 60)) && (euclidean_dist(ball.pos,robotPos) < distGiro))
    {
        rVel = -velGiroLado*vMax;
        lVel = velGiroLado*vMax;
    }

    // Função para fazer o robô girar na linha de fundo de ataque
    if ((ball.pos.y > (centroidAtk.y + 40)) && ((euclidean_dist(ball.pos, robotPos)) < distGiro) && (fabs(ball.pos.x - centroidAtk.x) < 15))
    {
        rVel = velGiroLado*vMax;
        lVel = -velGiroLado*vMax;
    }
    else if ((ball.pos.y < (centroidAtk.y - 40)) && (euclidean_dist(ball.pos, robotPos) < distGiro) && (fabs(ball.pos.x - centroidAtk.x) < 15))
    {
        rVel = -velGiroLado*vMax;
        lVel = velGiroLado*vMax;
    }

    // Função para fazer o robô girar na linha de fundo de defesa
    if ((ball.pos.y > (centroidDef.y + 35)) && ((euclidean_dist(ball.pos, robotPos)) < distGiro) && (fabs(ball.pos.x - centroidDef.x) < 15))
    {
        rVel = velGiroLado*vMax;
        lVel = -velGiroLado*vMax;
    }
    else if ((ball.pos.y < (centroidDef.y - 35)) && (euclidean_dist(ball.pos, robotPos) < (distGiro && fabs(ball.pos.x - centroidDef.x) < 15)))
    {
        rVel = -velGiroLado*vMax;
        lVel = velGiroLado*vMax;
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
    float teempo = 0;
    float vTurn = 120*0.01;

    if(firstAceleration == false)
    {
        teempo = (float) (clock() - clockAceleration)/CLOCKS_PER_SEC;
        if(robotFunctions[indexRobot]->getAtkSituationTiro() == true)
        {
            if(vAtk < 1)
                vAtk = 1;
        }
        else
        {
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
    }

    if (euclidean_dist(ball.pos,robotPos) < 20){
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

        }
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
    }
}

void Mover::setDirection(bool d)
{
    robotDirection = d;
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
//    Point2f robotPos=teamRobot[indexRobot].getDataState().pos;
//    robotPos.x = 20;
//    robotPos.y = 20;
//    float robotAngle = teamRobot[indexRobot].getDataState().angle;
//    robotAngle = 0;

//    //Posição da meta
//    Point2f meta = robotFunctions[indexRobot]->getGoal();
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

void Mover::rotate_def (){

    Point2f robotPos=teamRobot[indexRobot].getDataState().pos;
    int distGiro;

    if (teamRobot[indexRobot].getFunction()==GOALKEEPER){
        distGiro=8;
    }
    else if (teamRobot[indexRobot].getFunction()==DEFENDER){
        distGiro=8;
    }

    if((robotPos.y > ball.pos.y) && (euclidean_dist(ball.pos,robotPos) < distGiro))
    {
        lVel = -velGiroGol*100;
        rVel = velGiroGol*100;
    }
    else if((robotPos.y < ball.pos.y) && (euclidean_dist(ball.pos,robotPos) < distGiro))
    {
        lVel = velGiroGol*100;
        rVel = -velGiroGol*100;
    }
}

void Mover::setIndex(int index)
{
    indexRobot = index;
}

float Mover::getKp()
{
    return kp;
}

void Mover::setKp(float a)
{
    kp = a;
}

float Mover::getKd()
{
    return kd;
}

void Mover::setKd(float a)
{
    kd = a;
}
