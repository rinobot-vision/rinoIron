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
        if(getPlot() == true)
            PlotPath(1, teamRobot[indexRobot]);
    }
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
//    dataState robot = teamRobot[indexRobot].getDataState();
//    goal = ball.pos;
//    thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);

    flagGoAhead = false;
    setStopOnGoal(false);
    airball=false;
    flagAvoidDefender = false;
    de = deW;
    Kr = KrW;
    dataState robot = teamRobot[indexRobot].getDataState();

    dataState goalk = teamRobot[indexRobot].getDataState();
    if (teamRobot[0].getFunction() == GOALKEEPER)
        goalk = teamRobot[0].getDataState();
    else if (teamRobot[1].getFunction() == GOALKEEPER)
        goalk = teamRobot[1].getDataState();
    else if (teamRobot[2].getFunction() == GOALKEEPER)
        goalk = teamRobot[2].getDataState();

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
        gSize = gSizeW;

    if(ball.pos.x < centroidDef.x + 24 && ball.pos.y < centroidDef.y + 40 && ball.pos.y > centroidDef.y - 40 && againstTheTeam == false )
    {
        goal.x = centroidDef.x + 24;
        goal.y = ball.pos.y;
        setStopOnGoal(true);
        airball = true;
    }
    else if(ball.pos.x < centroidDef.x + defenderLine)
    {
        if(ball.pos.y < centroidDef.y - 30)
        {
            goal.x = defenderLine - 3;
            goal.y = centroidDef.y - 25;
            setStopOnGoal(true);
        }
        if(ball.pos.y > centroidDef.y + 30)
        {
            goal.x = defenderLine + 3;
            goal.y = centroidDef.y + 25;
            setStopOnGoal(true);
        }

    }
    else
    {
        if (ball.pos.y < 15  || ball.pos.y > 115)
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
            minLimit.y = 50;
            maxLimit.x = centroidAtk.x;
            maxLimit.y = 80;

            Point2f ang_point;
            if(ball.pos.y < robot.pos.y)
                ang_point.y = robot.pos.y - fabs(cat_op);
            else
                ang_point.y = robot.pos.y + fabs(cat_op);

            ang_point.x = centroidAtk.x;

            if((ang_point.y < 80) && (ang_point.y > 50) && robot.pos.x < ball.pos.x)
            {
                thetaDir = (alpha*M_PI/180);
                atkPoint = ang_point;
                setAtkSituation(true);
                flagGoAhead = true;
                thetaDir = 0;
            }
            else
            {
                if(robot.pos.x < ball.pos.x)
                {
                    if(ang_point.y > 80)
                    {
                        //thetaDir = angleTwoPoints(ball.pos,minLimit)*PI/180;
                        atkPoint = minLimit;
                        thetaDir = 0;
                        atkPoint = centroidAtk;
                    }
                    else if(ang_point.y < 50)

                    {
                        //thetaDir = angleTwoPoints(ball.pos,maxLimit)*PI/180;
                        atkPoint = maxLimit;
                        thetaDir = 0;
                        atkPoint = centroidAtk;
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

            if(ball.pos.y >= 50 && ball.pos.y <= 80 && (robot.pos.y < 50 || robot.pos.y > 80))
            {
                thetaDir = 0;
            }
            if(againstTheTeam == false)
            {
                if(ball.pos.x > centroidAtk.x - 10)
                {
                    if(ball.pos.y > centroidDef.y - 65 && ball.pos.y < centroidDef.y - 20)
                    {
                        thetaDir = (40*PI/180);
                    }
                    if(ball.pos.y < centroidDef.y + 65 && ball.pos.y > centroidDef.y + 20)
                    {
                        thetaDir = (-40*PI/180);

                    }
                }
            }
            else
            {
                if(ball.pos.x < centroidAtk.x + 10)
                {
                    if(ball.pos.y > centroidDef.y - 65 && ball.pos.y < centroidDef.y - 20)
                    {
                        thetaDir = (140*PI/180);
                    }
                    if(ball.pos.y < centroidDef.y + 65 && ball.pos.y > centroidDef.y + 20)
                    {
                        thetaDir = (-140*PI/180);
                    }
                }
            }

        }
        if(ball.pos.y < 20  || ball.pos.y > 110)
        {
            if(robot.pos.x > ball.pos.x)
            {
                if(ball.pos.y < 35  && robot.pos.y < ball.pos.y)
                {
                    thetaDir = (-90 * PI / 180);
                    de = 0.7;
                    Kr = 0;
                }
                if(ball.pos.y > 95 && robot.pos.y > ball.pos.y)
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
                if(robot.pos.y<25||robot.pos.y>105)
                {
                    de = 1;
                    Kr = 0;

                }
            }

            //            else
            //            {
            //                de = deW;
            //                Kr = KrW;
            //            }

        }
        else
        {
            de = 1;
            Kr = 0;
            gSize = gSizeW;

            if(goal.y > 65)
                thetaDir = 90*PI/180;
            else
                thetaDir = -90*PI/180;

            if(ball.pos.x < 20)
            {
                if(goal.y > 65 && robot.pos.y < ball.pos.y)
                    thetaDir = angleTwoPoints(robot.pos,ball.pos)*PI/180;
                if(goal.y < 65 && robot.pos.y > ball.pos.y)
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

    //    if(getPenalty() == 1)
    //    {
    //        setKickState(true);
    //        setAtkSituation(false);
    //    }
    //    else {
    //        setKickState(false);
    //    }

    dataState robotAux;
    Point2f defender;
    if (teamRobot[0].getFunction() == DEFENDER)
        defender = teamRobot[0].getDataState().pos;
    else if (teamRobot[1].getFunction() == DEFENDER)
        defender = teamRobot[1].getDataState().pos;
    else if (teamRobot[2].getFunction() == DEFENDER)
        defender = teamRobot[2].getDataState().pos;

    //    int r = 2;
    //    float dist = euclidean_dist(robot.pos,ball.pos);
    //    if(dist <= 10 && robot.pos.x < ball.pos.x)
    //    {
    //        cout << "ok" << endl;
    //        goal.x = goal.x + r*cos(thetaDir);
    //        goal.y = goal.y + r*sin(thetaDir);
    //    }
    //    else {
    //        cout << "not ok" << endl;
    //    }

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
            //            if(flagGoAhead)
            //                thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);
            //            elseHIG
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
//    cout << "META STRIKER: (" << goal.x << "," << goal.y << ")" << endl;

}

void GameFunctions::defender()
{
    flagAvoidGoalkepper = false;
    flagKickBall = false;
    dataState robot = teamRobot[indexRobot].getDataState();
    dataState goalk = teamRobot[indexRobot].getDataState();
    if (teamRobot[0].getFunction() == GOALKEEPER)
        goalk = teamRobot[0].getDataState();
    else if (teamRobot[1].getFunction() == GOALKEEPER)
        goalk = teamRobot[1].getDataState();
    else if (teamRobot[2].getFunction() == GOALKEEPER)
        goalk = teamRobot[2].getDataState();
    if (ball.pos.x > centroidDef.x + defenderLine)
    {
        goal.x = centroidDef.x + defenderLine;
        goal.y = ball.pos.y;
        if(robot.pos.x < centroidDef.x + defenderLine - 15)
            if(strategy == NS)
                flagAvoidGoalkepper = false;
            else
                flagAvoidGoalkepper = true;
        else
            flagAvoidGoalkepper = false;
    }

    else
    {
        if(ball.pos.y < 30)
        {
            goal = ball.pos;
            flagKickBall = true;
            gSize = gSizeW;
            de = deW;
            Kr = KrW;
            thetaDir = 55.0;
            //            cout << "UM" << endl;
        }
        else if(ball.pos.y > 100)
        {
            goal = ball.pos;
            flagKickBall = true;
            gSize = gSizeW;
            de = deW;
            Kr = KrW;
            thetaDir = -55.0;
            //            cout << "DOIS" << endl;
        }

        else
        {
            goal.x = defenderLine + centroidDef.x;
            goal.y = ball.pos.y;
            //            cout << "TRES" << endl;
            if(ball.pos.x < centroidDef.x + 15)
            {
                if(ball.pos.y > centroidDef.y)
                {
                    goal.x = centroidDef.x + 37;
                    goal.y = centroidDef.y + 42;
                }
                else
                {
                    goal.x = centroidDef.x + 37;
                    goal.y = centroidDef.y - 42;
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
                //                cout << "uni" << endl;
            }
            else
            {
                thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);
                //                cout << "duni" << endl;
            }
        }
    }

}


void GameFunctions::goalkeeper()
{

    if(ball.pos.x < centroidDef.x + 90)
    {
        if(ball.pos.y < centroidDef.y - 20)
        {
            goal.x = centroidDef.x + 5;
            goal.y = centroidDef.y - 15;
        }
        else if(ball.pos.y > centroidDef.y + 20)
        {
            goal.x = centroidDef.x + 5;
            goal.y = centroidDef.y + 15;
        }
        else
        {
            goal.x = centroidDef.x + 5;
            goal.y = ball.pos.y;
        }
    }
    else
    {
        goal.x = centroidDef.x + 5;
        goal.y = centroidDef.y;
    }

    thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);
    if(tiroMetaSituation == true)
    {
        kickBall();
    }
    cout << "META GOLEIRO: (" << goal.x << "," << goal.y << ")" << endl;

}

void GameFunctions::fake9()
{
    flagAvoidDefender = false;
    flagAvoidGoalkepper = false;
    float distx = 20;
    float disty = 20;
    dataState robot = teamRobot[indexRobot].getDataState();
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

    Point2f defenderMeta;
    defenderMeta.x = centroidDef.x + defenderLine;
    defenderMeta.y = ball.pos.y;

    k_larg = 0.06;
    goal.x = killer.pos.x - distx;

    if(goal.x < defenderMeta.x)
    {
        k_larg = 0.06;
        goal.x = defenderMeta.x + 10;
    }

    if(ball.pos.y > (centroidAtk.y + 35))
        goal.y = centroidAtk.y + 35;
    else if(ball.pos.y < (centroidAtk.y - 35))
        goal.y = centroidAtk.y - 35;
    else
    {
        if(ball.pos.y > centroidAtk.y )
        {
            goal.y = ball.pos.y - disty;
        }
        else
        {
            goal.y = ball.pos.y + disty;
        }
    }


    if(flagAvoidGoalkepper)
    {
        thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), goalk.pos);
    }
    else
    {
        if(euclidean_dist(robot.pos,defender) < 15)
        {
            k_larg = 0.04;
            flagAvoidDefender = true;
            thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), defender);
        }else
            if(robot.pos.x < killer.pos.x - 10)
                thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos,goal);
            else
                thePhi = repulsiveMath(teamRobot[indexRobot].getDataState(), ball.pos);
    }
//    cout << "META FAKE9: (" << goal.x << "," << goal.y << ")" << endl;

}


void GameFunctions::midfield()
{
    dataState robot = teamRobot[indexRobot].getDataState();
    float time;
    time = 0.2;

    if( euclidean_dist(robot.pos, ball.pos) <= 15)
    {
        time = 0.01;
    }

    //time = 0.0001*(robot.pos.y - ball.pos.y)*ball.vel.y;
    setAtkSituation(false);
    setCrossing(false);
    //    cout << "time: "<< time << endl;

    Point2f ballPrev= Point2f(ball.pos.x + time*ball.vel.x, ball.pos.y + time*ball.vel.y);

    Point2f aux, minLimit, maxLimit;
    aux.x = centroidAtk.x;
    aux.y = robot.pos.y;
    float theta = angleTwoPoints(ball.pos,robot.pos);
    float alpha = angleTwoPoints(robot.pos,ball.pos);
    float cat_ad = euclidean_dist(robot.pos,aux);
    float cat_op = cat_ad*tan(theta*PI/180);

    minLimit.x = centroidAtk.x;
    minLimit.y = 35;
    maxLimit.x = centroidAtk.x;
    maxLimit.y = 85;

    Point2f ang_point;
    if(ball.pos.y < robot.pos.y)
        ang_point.y = robot.pos.y - fabs(cat_op);
    else
        ang_point.y = robot.pos.y + fabs(cat_op);

    ang_point.x = centroidAtk.x;

    if((ang_point.y < 85) && (ang_point.y > 45) && (robot.pos.x < ball.pos.x))
    {
        setAtkSituation(true);
    }
    if((robot.pos.x < ball.pos.x) && (ball.pos.y < 110) && (ball.pos.y > 20))
    {
        goal = ballPrev;
        setCrossing(true);
        if(fedUp)
        {
            speedmf = clock();
            fedUp = false;
        }
    }
    else
    {
        if(ball.pos.x < robot.pos.x)
        {

        }
        else
        {
            if(ball.pos.y>centroidAtk.y)
            {
                goal.x=centroidAtk.x-45;
                goal.y=centroidAtk.y+20;
            }
            else
            {
                goal.x=centroidAtk.x-45;
                goal.y=centroidAtk.y-20;
            }
            //goal.x = centroidAtk.x - 60;
            //goal.y = centroidAtk.y;
        }
    }

    //        gSize = gSizeW;
    //        de = deW;
    //        Kr = KrW;
    //        thetaDir = 0;

    //          univectorField(robot, Point2f(0,0));
    thePhi = angleTwoPoints(robot.pos,goal);
//    cout << "META MIELDFIELD: (" << goal.x << "," << goal.y << ")" << endl;

}

void GameFunctions::wing()
{
    setStopOnGoal(false);
    dataState robot = teamRobot[indexRobot].getDataState();
    float time;
    if(ball.vel.x > 0)
        time = 0.05;
    else
        time = 0;

    //    if (robot.pos.x > goal.x)
    //        gSize = 0;
    //    else
    gSize = gSizeW;

    de = deW;
    Kr = KrW;


    Point2f ballPrev= Point2f(ball.pos.x + time*ball.vel.x, ball.pos.y + time*ball.vel.y);
    goal = ballPrev;

    if(ball.pos.y > centroidAtk.y + 35 || ball.pos.y < centroidAtk.y - 35)
    {
        Kr = 0;
        if(ball.pos.y > centroidAtk.y + 35)
        {
            if(ball.pos.y > centroidAtk.y + 60)
                thetaDir = (25*PI/180);
            else
                thetaDir = (0*PI/180);
        }
        else if(ball.pos.y < centroidAtk.y - 35)
        {
            if(ball.pos.y < centroidAtk.y - 60)
                thetaDir = (-25*PI/180);
            else
                thetaDir = (0*PI/180);
        }

        if(robot.pos.x > ball.pos.x)
        {
            de = 1;
            Kr = 0;
            if(ball.pos.y < 35  && robot.pos.y < ball.pos.y)
            {
                thetaDir = (-90 * PI / 180);
            }
            if(ball.pos.y > 95 && robot.pos.y > ball.pos.y)
            {
                thetaDir = (90 * PI / 180);
            }
        }
    }
    else
    {
        setStopOnGoal(true);
        goal.x = centroidAtk.x - 50;
        if(robot.pos.y > centroidAtk.y)
        {
            goal.y = centroidAtk.y + 50;
        }
        else
        {
            goal.y = centroidAtk.y - 50;
        }
        thetaDir = angleTwoPoints(robot.pos,goal)*PI/180;
    }
    univectorField(robot, Point2f(0,0));
//    cout << "META WING: (" << goal.x << "," << goal.y << ")" << endl;

}

void GameFunctions::volante()
{
    if((ball.pos.x > 0) && (ball.pos.y > 0))
    {
        if (centroidDef.x < centroidAtk.x)
        {
            if (ball.pos.x > (centroidDef.x + volanteLine))
            {
                goal.x = centroidDef.x + volanteLine;
                goal.y = ball.pos.y; //centroidDef.y;
            }
            else
            {
                if(ball.pos.y < (centroidDef.y - 35))
                {
                    goal.x = centroidDef.x + 8;
                    goal.y = centroidDef.y - 45;
                    thetaDir = PI;
                }
                else if(ball.pos.y > (centroidDef.y + 35))
                {
                    goal.x = centroidDef.x + 8;
                    goal.y = centroidDef.y + 45;
                    thetaDir = PI;
                }
                else
                {
                    goal.x = centroidDef.x + volanteLine;
                    goal.y = centroidDef.y;
                }
            }

        }
    }
    else if (centroidDef.x >= centroidAtk.x)
    {
        if (ball.pos.x < (centroidDef.x - volanteLine))
        {
            goal.x = centroidDef.x - volanteLine;
            goal.y = ball.pos.y; //centroidDef.y;
        }
        else
        {
            if(ball.pos.y < (centroidDef.y - 35))
            {
                goal.x = centroidDef.x - 8;
                goal.y = centroidDef.y - 45;
                thetaDir = 0;
            }
            else if(ball.pos.y > (centroidDef.y + 35))
            {
                goal.x = centroidDef.x - 8;
                goal.y = centroidDef.y + 45;
                thetaDir = 0;
            }
            else
            {
                goal.x = centroidDef.x - volanteLine;
                goal.y = centroidDef.y;
            }
        }
    }
}


void GameFunctions::libero()
{
    dataState robot = teamRobot[indexRobot].getDataState();
    Point2f goal;
    float time = 0.05;
    Point2f ballPrev= Point2f(ball.pos.x + time*ball.vel.x, ball.pos.y + time*ball.vel.y);
    int distbola = 30;
    if(ball.pos.x > defenderLine)
    {
        goal.y = ballPrev.y;
        if(ball.vel.x >= 0)
        {
            goal.x = ballPrev.x - distbola;
            lastX = goal.x;
            //cout<< "lastX: "<<lastX<<endl;
        }
        else
        {
            if(ball.pos.x >= robot.pos.x)
            {
                goal.x = lastX;
            }
            else
            {
                goal.x = ball.pos.x - distbola;
            }
        }

     thetaDir = PI/2;
     thePhi = angleTwoPoints(robot.pos, goal);
    }
    else
    {
        if(ball.pos.x <= 25)
        {
            if(ball.pos.y <= 30)
            {
                goal.x = 4;
                goal.y = 25;
            }
            if(ball.pos.y >= 100)
            {
                goal.x = 4;
                goal.y = 105;
            }
            else
            {
                goal.x = 30;
                goal.y = 25;
            }
            thetaDir = PI/2;
            thePhi = angleTwoPoints(robot.pos, goal);
        }
        if(ball.pos.x > 25 && ball.pos.x <= defenderLine)
        {
            goal = ball.pos;
            thetaDir = 0;
            univectorField(robot, Point2f(0,0));
        }
    }

}

void GameFunctions::offdefender()
{
    dataState robot = teamRobot[indexRobot].getDataState();
    goal.x = centroidDef.x + offLine;
    goal.y = ball.pos.y;
    thePhi = angleTwoPoints(teamRobot[indexRobot].getDataState().pos, goal);
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
        case WING:
            ang = (ajustaAngulo(univectorFieldForPlot(virtualRobot,enemy)))*PI/180;
            break;
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

void GameFunctions::AtkPath(robot robot)
{
    float ang;
    dataState virtualRobot;
    virtualRobot.pos = robot.getDataState().pos;
    int r = 1;
    int cont = 0;
    int state = 0;
    Point2f enemy = Point2f(0,0);

    while((euclidean_dist(virtualRobot.pos,ball.pos) > 5)&&(cont < 215))//(fabs(virtualRobot.pos.x - goal.x) > 1) && (fabs(virtualRobot.pos.y - goal.y) > 1))
    {
        AddAtkPoint(virtualRobot.pos);
        ang = (ajustaAngulo(univectorFieldForPlot(virtualRobot,enemy)))*PI/180;
        virtualRobot.pos.x = virtualRobot.pos.x + r*cos(ang);
        virtualRobot.pos.y = virtualRobot.pos.y + r*sin(ang);
        cont++;
    }
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
void GameFunctions::setTiroMeta(bool state)
{
    tiroMetaSituation = state;
}

bool GameFunctions::getTiroMeta()
{
    return tiroMetaSituation;
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
