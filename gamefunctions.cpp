#include "gamefunctions.h"

GameFunctions::GameFunctions()
{

}

GameFunctions::~GameFunctions(){

}

void GameFunctions::run()
{
    if((int)teamRobot.size() > indexRobot)
    {
        switch (teamRobot[indexRobot].getFunction()) {
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
        case DEFENDERSHIELD:
            defendershield();
            break;
        case DEFENDER2:
            defender2();
            break;
        default:
            break;
        }
        if(getPlot() == true)
            PlotPath(1, teamRobot[indexRobot]);
    }
}

float GameFunctions::getgSizeW()
{
    return gSizeW;
}

void GameFunctions::setgSizeW(float a)
{
    gSizeW = a;
}

float GameFunctions::getdeW()
{
    return deW;
}

void GameFunctions::setdeW(float a)
{
    deW = a;
}

float GameFunctions::getkrW()
{
    return KrW;
}

void GameFunctions::setkrW(float a)
{
    KrW = a;
}

float GameFunctions::getkLarg()
{
    return k_larg;
}

void GameFunctions::setkLarg(float a)
{
    k_larg = a;
}


void GameFunctions::setRobots(vector<robot> robots)
{
    teamRobot = robots;
}

void GameFunctions::setAreas(Point2f def, Point2f atk)
{
    if(!againstTheTeam){
        centroidDef = def;
        centroidAtk = atk;
    }
    else{
        centroidDef = atk;
        centroidAtk = def;
    }
}

void GameFunctions::setAgainstTheTeam(bool a)
{
    againstTheTeam = a;
}


void GameFunctions::setBall(dataState b)
{
    ball = b;
}

Point2f GameFunctions::getGoal()
{
    return goal;
}

float GameFunctions::getDirection()
{
    return thePhi;
}

float GameFunctions::getDefenderLine()
{
    return defenderLine;
}

float GameFunctions::getVolanteLine()
{
    return volanteLine;
}

void GameFunctions::setDefenderLine(float possld)
{
    defenderLine = possld;
}

void GameFunctions::setAtkSituation(bool a)
{
    atkSituation = a;
}

void GameFunctions::setAtkSituationTiro(bool a)
{
    atkSituationTiro = a;
}

void GameFunctions::setAtkSituationInv(bool a)
{
    atkSituationInv = a;
}

bool GameFunctions::getAtkSituation()
{
    return atkSituation;
}

bool GameFunctions::getAtkSituationTiro()
{
    return atkSituationTiro;
}

void GameFunctions::setCrossing(bool a)
{
    flagCrossing = a;
}

bool GameFunctions::getCrossing()
{
    return flagCrossing;
}

bool GameFunctions::getAtkSituationInv()
{
    return atkSituationInv;
}

void GameFunctions::setStopOnGoal(bool a)
{
    flagStopOnGoal = a;
}

bool GameFunctions::getStopOnGoal()
{
    return flagStopOnGoal;
}

bool GameFunctions::getAgainsTheTeam()
{
    return againstTheTeam;
}

void GameFunctions::striker()
{
    // Quando a bola estiver no ataque e o atacante estiver atrás do defender, ele desvia dele.

    flagGoAhead = false;
    setStopOnGoal(false);
    airball=false;
    flagAvoidDefender = false;
    de = deW;
    Kr = KrW;
    dataState robot = teamRobot[indexRobot].getDataState();
    dataState goalk = teamRobot[indexRobot].getDataState();

    for(int i = 0; i < 5; i++)
    {
        if (teamRobot[i].getFunction() == GOALKEEPER)
        {
            goalk = teamRobot[i].getDataState();
            break;
        }
    }

    float time;
    if(ball.vel.x > 0 && euclidean_dist(teamRobot[indexRobot].getDataState().pos, ball.pos)>12)
        time = 0.07;
    else
        time = 0;

    Point2f ballPrev= Point2f(ball.pos.x + time*ball.vel.x, ball.pos.y + time*ball.vel.y);
    goal = ballPrev;

    if (robot.pos.x > goal.x)
        gSize = 0;
    else
        gSize = 0.6;

    if(ball.pos.x < centroidDef.x + 35 && ball.pos.y < centroidDef.y + 40 && ball.pos.y > centroidDef.y - 40 && againstTheTeam == false )
    {
        if(ball.pos.y > centroidDef.y)
        {
            goal.x = centroidDef.x + 40;
            goal.y = centroidDef.y + 40;
            setStopOnGoal(true);
            airball = true;
        }

        else
        {
            goal.x = centroidDef.x + 40;
            goal.y = centroidDef.y - 40;
            setStopOnGoal(true);
            airball = true;
        }
    }

    else
    {
        goal.x = ball.pos.x;
        goal.y = ball.pos.y;
        thetaDir = 0;

        if (ball.pos.y < 15  || ball.pos.y > 165)
        {
            if(ball.pos.y < centroidAtk.y)
                if(againstTheTeam == false)
                    thetaDir = (-25*PI/180);
                else
                    thetaDir = (-135*PI/180);
            if(ball.pos.y > centroidAtk.y)
                if(againstTheTeam == false)
                    thetaDir = (25*PI/180);
                else
                    thetaDir = (-225*PI/180);
        }
        else
        {
            Point2f aux, minLimit, maxLimit;
            aux.x = centroidAtk.x;
            aux.y = robot.pos.y;
            float theta = angleTwoPoints(ball.pos,robot.pos);
            float alpha = angleTwoPoints(robot.pos,ball.pos);
            float cat_ad = euclidean_dist(robot.pos,aux);
            float cat_op = cat_ad*tan(theta*PI/180);

            minLimit.x = centroidAtk.x;
            minLimit.y = 75;
            maxLimit.x = centroidAtk.x;
            maxLimit.y = 105;

            Point2f ang_point;
            if(ball.pos.y < robot.pos.y)
                ang_point.y = robot.pos.y - fabs(cat_op);
            else
                ang_point.y = robot.pos.y + fabs(cat_op);

            ang_point.x = centroidAtk.x;

            if((ang_point.y < 105) && (ang_point.y > 75) && robot.pos.x < ball.pos.x)
            {
                thetaDir = (alpha*M_PI/180);
                atkPoint = ang_point;
                setAtkSituation(true);
                flagGoAhead = true;
                //thetaDir = 0;
            }
            else
            {
                if(robot.pos.x < ball.pos.x)
                {
                    if(ang_point.y > 105)
                    {
                        thetaDir = angleTwoPoints(ball.pos,maxLimit)*PI/180;
                        atkPoint = maxLimit;
                        //thetaDir = 0;
                        //atkPoint = centroidAtk;
                    }
                    else if(ang_point.y < 75)

                    {
                        thetaDir = angleTwoPoints(ball.pos,minLimit)*PI/180;
                        atkPoint = minLimit;
                        //thetaDir = 0;
                        //atkPoint = centroidAtk;
                    }
                    else
                    {
                        //thetaDir = angleTwoPoints(ball.pos,centroidAtk)*PI/180;
                        thetaDir = 0;
                        atkPoint = centroidAtk;
                    }
                }
                else
                {
                    thetaDir = 0;
                }
                setAtkSituation(false);
            }

            if(ball.pos.y >= 75 && ball.pos.y <= 105 && (robot.pos.y < 75 || robot.pos.y > 105))
            {
                thetaDir = 0;
            }
            if(againstTheTeam == false)
            {
                if(ball.pos.x > centroidAtk.x - 10)
                {
                    if(ball.pos.y > centroidDef.y - 90 && ball.pos.y < centroidDef.y - 20)
                    {
                        thetaDir = (40*PI/180);
                    }
                    if(ball.pos.y < centroidDef.y + 90 && ball.pos.y > centroidDef.y + 20)
                    {
                        thetaDir = (-40*PI/180);

                    }
                }
            }
            else
            {
                if(ball.pos.x < centroidAtk.x + 10)
                {
                    if(ball.pos.y > centroidDef.y - 90 && ball.pos.y < centroidDef.y - 20)
                    {
                        thetaDir = (140*PI/180);
                    }
                    if(ball.pos.y < centroidDef.y + 90 && ball.pos.y > centroidDef.y + 20)
                    {
                        thetaDir = (-140*PI/180);
                    }
                }
            }

        }
        if(ball.pos.y < 20  || ball.pos.y > 160)
        {
            if(robot.pos.x > ball.pos.x)
            {
                if(ball.pos.y < 35  && robot.pos.y < ball.pos.y)
                {
                    thetaDir = (-90 * PI / 180);
                    de = 0.7;
                    Kr = 0;
                }
                if(ball.pos.y > 145 && robot.pos.y > ball.pos.y)
                {
                    thetaDir = (90 * PI / 180);
                    de = 0.7;
                    Kr = 0;
                }
            }
        }
    }

    if(againstTheTeam == false)
    {
        if(goal.x > centroidDef.x + 35)
        {
            if(goal.x > centroidAtk.x-10)
            {
                if(robot.pos.x > goal.x)
                {
                    de = 0.7;
                    Kr = 0;
                    thetaDir = 0;
                }
            }
            else
            {
                if(robot.pos.y<25||robot.pos.y>155)
                {
                    de = 1;
                    Kr = 0;

                }
            }

        }
        else
        {
            de = 1;
            Kr = 0;
            gSize = gSizeW;

            if(goal.y > 90)
                thetaDir = 90*PI/180;
            else
                thetaDir = -90*PI/180;

            if(ball.pos.x < 20)
            {
                if(goal.y > 90 && robot.pos.y < ball.pos.y)
                    thetaDir = angleTwoPoints(robot.pos,ball.pos)*PI/180;
                if(goal.y < 90 && robot.pos.y > ball.pos.y)
                    thetaDir = angleTwoPoints(robot.pos,ball.pos)*PI/180;

            }
        }
    }
    else
    {
        if(goal.x < centroidDef.x - 35){
            de = deW;
            Kr = KrW;
        }
        else{
            de = 1;
            Kr = 0;
        }
    }

    dataState robotAux;
    Point2f defender;

    // testar se o robô não está entrando na área grande

    for(int i = 0; i < 5; i++)
    {
        if (teamRobot[i].getFunction() == DEFENDER){
            defender = teamRobot[i].getDataState().pos;
            break;
        }
    }

    if (ball.pos.x < centroidDef.x + defenderLine)
    {
        if(ball.pos.y < centroidDef.y - 35)
        {
            if(robot.pos.x < centroidDef.x + defenderLine)
            {
                if(robot.pos.y > goalk.pos.y - 10)
                    flagAvoidGoalkepper = true;
                else
                    flagAvoidGoalkepper = false;
            }
            else
            {
                flagAvoidGoalkepper = false;
            }
        }
        else if(ball.pos.y > centroidDef.y + 35)
        {
            if(robot.pos.x < centroidDef.x + defenderLine)
            {
                if(robot.pos.y < goalk.pos.y + 10)
                    flagAvoidGoalkepper = true;
                else
                    flagAvoidGoalkepper = false;
            }
            else
            {
                flagAvoidGoalkepper = false;
            }
        }
    }

    if(flagAvoidGoalkepper)
    {
        k_larg = 0.08;
        thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), goalk.pos);
    }
    else
    {
        if(euclidean_dist(robot.pos,defender) < 15 && euclidean_dist(ball.pos,defender) > 15)
        {
            flagAvoidDefender = true;
            k_larg = 0.06;
            thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), defender);
        }
        else
        {
            if(flagStopOnGoal)
                thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos, goal);
            else
                univectorField(teamRobot[indexRobot].getDataState(), defender);
        }
    }
    if(penaultyPermission == true || freeBallSituation == true)
    {
        kickBall();
        setAtkSituation(true);
    }

    if (tempoRepulsive == 0)
    {
        ClockStartR = clock();
        tempoStopRepulsive = 0;
        flagGrab = false;
        StrikeRepulsive = robot.pos;
    }

    tempoRepulsive = (double) (clock() - ClockStartR)/CLOCKS_PER_SEC;

    if (tempoRepulsive >= 3){
        if (euclidean_dist(StrikeRepulsive, robot.pos) <= 5 || flagGrab == true)
        {
            //cout << "REPULSIVE" << endl;
            k_larg = 0.01;
            thePhi = ajustaAngulo(thePhi + 90);
            //cout << "thePHI: " << thePhi << endl;
            if(flagGrab == false){
                //cout << "iniciou stop clock" << endl;
                ClockStopR = clock();
            }
            flagGrab = true;
        }
        else
        {
            tempoRepulsive = 0;
        }
    }
    if(flagGrab == true){
        tempoStopRepulsive = (double) (clock() - ClockStopR)/CLOCKS_PER_SEC;
    }

    if (tempoStopRepulsive >= 0.5)
    {
        //cout << "REINICIA REPULSIVE" << endl;
        tempoRepulsive = 0;
        tempoStopRepulsive = 0;

        flagGrab = false;
    }

}

void GameFunctions::defendershield()
{
    dataState defendershield = teamRobot[indexRobot].getDataState();

    for(int i = 0; i < 5; i++)
    {
        if (teamRobot[i].getFunction() == DEFENDERSHIELD)
        {
            defendershield = teamRobot[i].getDataState();
            break;
        }
    }

    float ballAngle = angleTwoPoints(centroidDef,ball.pos);
    bool insideElipse = false;
    goal.x = centroidDef.x + (28*cos(ballAngle));
    goal.y = centroidDef.y + (32.5*sin(ballAngle));

    if (ball.pos.y < centroidDef.y)
    {
        if (ball.pos.x < goal.x && ball.pos.y > goal.y) //tolerância da elipse (+-4)
        {
            goal.x = centroidDef.x + 20;
            goal.y = centroidDef.y + 30;
            insideElipse = true;
        }
    }

    else
    {
        if (ball.pos.x < goal.x && ball.pos.y < goal.y) //tolerância da elipse (+-4)
        {
            goal.x = centroidDef.x + 20;
            goal.y = centroidDef.y - 30;
            insideElipse = true;
        }

        else
        {
            insideElipse = false;
        }
    }

    if(insideElipse == false && ball.pos.x < defendershield.pos.x)
    {
        goal.x = centroidDef.x + (28*cos(ballAngle));
        goal.y = centroidDef.y + (32.5*sin(ballAngle));
    }

    thePhi = angleTwoPoints(defendershield.pos, goal);
}

void GameFunctions::defender()
{
    flagAvoidGoalkepper = false;
    flagKickBall = false;
    dataState robot = teamRobot[indexRobot].getDataState();
    dataState goalk = teamRobot[indexRobot].getDataState();

    for(int i = 0; i < 5; i++)
    {
        if (teamRobot[i].getFunction() == GOALKEEPER)
        {
            goalk = teamRobot[i].getDataState();
            break;
        }
    }

    if (ball.pos.x > centroidDef.x + defenderLine)
    {
        goal.x = centroidDef.x + defenderLine;
        goal.y = ball.pos.y;
    }

    else
    {
        if(ball.pos.y < 50 && ball.pos.x < 50)
        {
            flagKickBall = true;
            gSize = gSizeW;
            de = deW;
            Kr = KrW;
            thetaDir = 55.0;
        }

        else if(ball.pos.y > 130 && ball.pos.x < 50)
        {
            flagKickBall = true;
            gSize = gSizeW;
            de = deW;
            Kr = KrW;
            thetaDir = -55.0;
        }

        else
        {
            goal.x = ball.pos.x;
            goal.y = ball.pos.y;
            thetaDir = 0;

            if(ball.pos.x < centroidDef.x + 35 && ball.pos.y < 130 && ball.pos.y > 50)
            {
                if(ball.pos.y > centroidDef.y)
                {
                    goal.x = centroidDef.x + 40;
                    goal.y = centroidDef.y + 45;
                }
                else
                {
                    goal.x = centroidDef.x + 40;
                    goal.y = centroidDef.y - 45;
                }
            }
        }
    }


    if(flagAvoidGoalkepper == true)
    {
        k_larg = 0.1;
        thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), goalk.pos);
    }
    else if(flagAvoidGoalkepper == false)
    {
        if(flagAvoidBall == true)
        {
            k_larg = 0.1;
            thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), ball.pos);
        }
        else
        {
            if(flagKickBall)
            {
                univectorField(robot, Point2f(0,0));
            }
            else
            {
                thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);
            }
        }
    }
}

void GameFunctions::defender2()
{
    flagAvoidGoalkepper = false;
    flagKickBall = false;
    dataState robot = teamRobot[indexRobot].getDataState();
    dataState goalk = teamRobot[indexRobot].getDataState();

    for(int i = 0; i < 5; i++)
    {
        if (teamRobot[i].getFunction() == GOALKEEPER)
        {
            goalk = teamRobot[i].getDataState();
            break;
        }
    }

    if (ball.pos.x > centroidDef.x + defenderLine2)
    {
        goal.x = centroidDef.x + defenderLine2;
        goal.y = ball.pos.y;
    }

    else
    {
        if(ball.pos.y < 50 && ball.pos.x < 50)
        {
            flagKickBall = true;
            gSize = gSizeW;
            de = deW;
            Kr = KrW;
            thetaDir = 55.0;
        }

        else if(ball.pos.y > 130 && ball.pos.x < 50)
        {
            flagKickBall = true;
            gSize = gSizeW;
            de = deW;
            Kr = KrW;
            thetaDir = -55.0;
        }

        else
        {
            goal.x = ball.pos.x;
            goal.y = ball.pos.y;
            thetaDir = 0;

            if(ball.pos.x < centroidDef.x + 35 && ball.pos.y < 130 && ball.pos.y > 50)
            {
                if(ball.pos.y > centroidDef.y)
                {
                    goal.x = centroidDef.x + 40;
                    goal.y = centroidDef.y + 45;
                }
                else
                {
                    goal.x = centroidDef.x + 40;
                    goal.y = centroidDef.y - 45;
                }
            }
        }
    }


    if(flagAvoidGoalkepper == true)
    {
        k_larg = 0.1;
        thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), goalk.pos);
    }
    else if(flagAvoidGoalkepper == false)
    {
        if(flagAvoidBall == true)
        {
            k_larg = 0.1;
            thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), ball.pos);
        }
        else
        {
            if(flagKickBall)
            {
                univectorField(robot, Point2f(0,0));
            }
            else
            {
                thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);
            }
        }
    }
}

void GameFunctions::goalkeeper()
{
    float distXballgkp;
    float previsionTime;
    float previsionYball;
    distXballgkp = ball.pos.x - (centroidDef.x +11);
    if(fabs(ball.vel.x ) < 30)
    {
        previsionTime = 0;
    }
    else
    {
        previsionTime = (distXballgkp / ball.vel.x);
    }

    previsionYball = (ball.vel.y * previsionTime) + ball.pos.y;
    Point2f defenderPos;

    int defenderindex;
    for(int i = 0; i < 5; i++)
    {
        if (teamRobot[i].getFunction() == DEFENDER)
        {
            defenderPos = teamRobot[i].getDataState().pos;
            defenderindex = i;
            break;
        }
    }

    if(ball.pos.x < centroidDef.x + 110)
    {
        if(previsionYball < centroidDef.y - 20)
        {
            goal.x = centroidDef.x + 7;
            goal.y = centroidDef.y - 15;
        }
        else if(previsionYball> centroidDef.y + 20)
        {
            goal.x = centroidDef.x + 7;
            goal.y = centroidDef.y + 15;
        }
        else
        {
            goal.x = centroidDef.x + 7;
            goal.y = ball.pos.y;
        }
    }
    else
    {
        goal.x = centroidDef.x + 7;
        goal.y = centroidDef.y;
    }

        if(ball.pos.x < teamRobot[indexRobot].getDataState().pos.x - 4)
        {
            if(ball.pos.x < centroidDef.x+15)
            {
                k_larg = 0.045;
            }
            else
            {
                k_larg = 0.04;
            }
            thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), ball.pos);
        }
        else if (defenderPos.x < teamRobot[indexRobot].getDataState().pos.x)
        {
            if(ball.pos.x < centroidDef.x+15)
            {
                k_larg = 0.045;
            }
            else
            {
                k_larg = 0.04;
            }
            thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), defenderPos);
        }
        else
        {
            thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);
        }
    //cout<<" Meta x = "<<goal.x<<" Meta y = "<< goal.y<<endl;
}

void GameFunctions::fake9()
{
    flagAvoidDefender = false;

    float distx = 25;
    float disty = 25;
    dataState robot = teamRobot[indexRobot].getDataState();
    dataState killer = teamRobot[indexRobot].getDataState();

    for(int i = 0; i < 5; i++)
    {
        if (teamRobot[i].getFunction() == STRIKER)
        {
            killer = teamRobot[i].getDataState();
            break;
        }
    }

    Point2f defender;

    for(int i = 0; i < 5; i++)
    {
        if (teamRobot[i].getFunction() == DEFENDER)
        {
            defender = teamRobot[i].getDataState().pos;
            break;
        }
    }

    Point2f defenderMeta;
    defenderMeta.x = centroidDef.x + defenderLine;
    defenderMeta.y = ball.pos.y;

    k_larg = 0.04;
    goal.x = killer.pos.x - distx;

    if(goal.x < defenderMeta.x)
    {
        k_larg = 0.04;
        goal.x = defenderMeta.x + 10;
    }

    if(ball.pos.y > centroidAtk.y)
    {
        goal.y = ball.pos.y - disty;
    }
    else
    {
        goal.y = ball.pos.y + disty;
    }

    if(euclidean_dist(robot.pos,defender) < 35)
    {
        k_larg = 0.05;
        flagAvoidDefender = true;
        thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), defender);
    }else{
        if(robot.pos.x < killer.pos.x - 10)
            thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);
        else
            thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), ball.pos);
    }
}



void GameFunctions::PlotPath(int i, robot robot)
{
    plotThePhi.clear();
    if(i == 0)
        atkPaths.clear();
    float ang, angStk;
    dataState virtualRobot, virtualStriker;
    virtualStriker.pos = robot.getDataState().pos;
    virtualRobot.pos = robot.getDataState().pos;
    int r = 5;
    int cont = 0;
    int state = 0;
    Point2f enemy;
    if (teamRobot[0].getFunction() == DEFENDER)
        enemy = teamRobot[0].getDataState().pos;
    else if (teamRobot[1].getFunction() == DEFENDER)
        enemy = teamRobot[1].getDataState().pos;
    else if (teamRobot[2].getFunction() == DEFENDER)
        enemy = teamRobot[2].getDataState().pos;
    dataState killer = teamRobot[indexRobot].getDataState();

    if (teamRobot[0].getFunction() == STRIKER)
        killer = teamRobot[0].getDataState();
    else if (teamRobot[1].getFunction() == STRIKER)
        killer = teamRobot[1].getDataState();
    else if (teamRobot[2].getFunction() == STRIKER)
        killer = teamRobot[2].getDataState();

    Point2f defender;
    if (teamRobot[0].getFunction() == DEFENDER)
        defender = teamRobot[0].getDataState().pos;
    else if (teamRobot[1].getFunction() == DEFENDER)
        defender = teamRobot[1].getDataState().pos;
    else if (teamRobot[2].getFunction() == DEFENDER)
        defender = teamRobot[2].getDataState().pos;

    dataState goalk = teamRobot[indexRobot].getDataState();

    if (teamRobot[0].getFunction() == GOALKEEPER)
        goalk = teamRobot[0].getDataState();
    else if (teamRobot[1].getFunction() == GOALKEEPER)
        goalk = teamRobot[1].getDataState();
    else if (teamRobot[2].getFunction() == GOALKEEPER)
        goalk = teamRobot[2].getDataState();

    while((euclidean_dist(virtualRobot.pos,goal) > 3)&&(cont < 215))//(fabs(virtualRobot.pos.x - goal.x) > 1) && (fabs(virtualRobot.pos.y - goal.y) > 1))
    {
        AddPlotPoint(virtualRobot.pos);
        switch (robot.getFunction())
        {
        case GOALKEEPER:
            ang = angleTwoPoints(virtualRobot.pos,goal)*PI/180;
            break;
//        case WING:
//            ang = (ajustaAngulo(univectorFieldForPlot(virtualRobot,enemy)))*PI/180;
//            break;
        case DEFENDER:
            if(flagAvoidGoalkepper)
            {
                ang = repulsiveMath(virtualRobot, goalk.pos)*PI/180;
            }
            else
            {
                if(flagAvoidBall)
                {
                    ang = repulsiveMath(virtualRobot, ball.pos)*PI/180;
                }
                else
                    ang = angleTwoPoints(virtualRobot.pos,goal)*PI/180;
            }
            break;
        case STRIKER:
            if(flagAvoidGoalkepper)
            {
                ang = repulsiveMath(virtualRobot, goalk.pos)*PI/180;
            }
            else
            {
                if(flagAvoidDefender)
                {
                    ang = repulsiveMath(virtualRobot, defender)*PI/180;
                }
                else
                    ang = (ajustaAngulo(univectorFieldForPlot(virtualRobot,enemy)))*PI/180;
            }
            break;
        case FAKE9:
            if(flagAvoidGoalkepper)
            {
                dataState goalk = teamRobot[indexRobot].getDataState();

                if (teamRobot[0].getFunction() == GOALKEEPER)
                    goalk = teamRobot[0].getDataState();
                else if (teamRobot[1].getFunction() == GOALKEEPER)
                    goalk = teamRobot[1].getDataState();
                else if (teamRobot[2].getFunction() == GOALKEEPER)
                    goalk = teamRobot[2].getDataState();

                ang = repulsiveMath(virtualRobot, goalk.pos)*PI/180;
            }
            else
            {
                if(flagAvoidDefender)
                {
                    Point2f defender;
                    if (teamRobot[0].getFunction() == DEFENDER)
                        defender = teamRobot[0].getDataState().pos;
                    else if (teamRobot[1].getFunction() == DEFENDER)
                        defender = teamRobot[1].getDataState().pos;
                    else if (teamRobot[2].getFunction() == DEFENDER)
                        defender = teamRobot[2].getDataState().pos;

                    ang = repulsiveMath(virtualRobot, defender)*PI/180;
                }
                else
                {
                    if(virtualRobot.pos.x < killer.pos.x - 10)
                        ang = angleTwoPoints(virtualRobot.pos,goal)*PI/180;
                    else
                        ang = repulsiveMath(virtualRobot, ball.pos)*PI/180;
                }
            }
            break;
        default:
            ang = angleTwoPoints(virtualRobot.pos,goal)*PI/180;
            break;
        }

        virtualRobot.pos.x = virtualRobot.pos.x + r*cos(ang);
        virtualRobot.pos.y = virtualRobot.pos.y + r*sin(ang);

        cont++;
    }
    //    if(robot.getFunction() == STRIKER)
    //        cout << "cont: " << cont << endl;
}

void GameFunctions::setPlot(bool state)
{
    plotState = state;
}

bool GameFunctions::getPlot()
{
    return plotState;
}

void GameFunctions::setKickState(bool state)
{
    kickPState = state;
}

bool GameFunctions::getKickState()
{
    return kickPState;
}

void GameFunctions::setPenalty(bool state)
{
    penaultyPermission = state;
}

bool GameFunctions::getPenalty()
{
    return penaultyPermission;
}

void GameFunctions::setStrategy(int s)
{
    strategy = s;
}

int GameFunctions::getStrategy()
{
    return strategy;
}

void GameFunctions :: kickBall()
{
    goal = ball.pos;
    thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,ball.pos);
}
void GameFunctions::setFreeBall(bool state)
{
    freeBallSituation = state;
}

bool GameFunctions::getFreeball()
{
    return freeBallSituation;
}

void GameFunctions::setlittleChute(bool state)
{
    littleChuteSituation = state;
}

bool GameFunctions::getlittleChute()
{
    return littleChuteSituation;
}

void GameFunctions::setIndex(int index)
{
    indexRobot = index;
}
