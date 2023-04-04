#include "gamewindow.h"
#include "QObject"

#define FOR_EACH_ROBOT_FUNCTIONS(methodcall)           \
    do{                                                \
    for(int i = 0; i < NUMROBOTS; i++){            \
    robotFunctions[i]->methodcall;             \
    }                                              \
    }while(0);

#define FOR_EACH_MOVER(methodcall)                     \
    do{                                                \
    for(int i = 0; i < NUMROBOTS; i++){            \
    mover[i]->methodcall;                      \
    }                                              \
    }while(0);


gamewindow::gamewindow()
{
    for(int i = 0; i < NUMROBOTS; i++)
    {
        robotFunctions[i] = new GameFunctions();
        mover[i] = new Mover();
    }

    for(int i = 0; i < NUMROBOTS; i++){
        mover[i]->setIndex(i);
        robotFunctions[i]->setIndex(i);
    }
}
void gamewindow::updateInfo(vector<robot> robots, vector<robot> enemy, Point2f def, Point2f atk, dataState ball, KNN k)
{
    deecision.setRobots(robots);
    deecision.setEnemy(enemy);
    deecision.setAreas(def,atk);
    deecision.setBall(ball);
    deecision.setKnnInformation(k);
    deecision.updateObjectives();

    vector<robot> ourTeam = deecision.getTeamRobots(), otherTeam = deecision.getEnemyRobots();
    Point2f centroidDef = deecision.getCentroidDef(), centroidAtk = deecision.getCentroidAtk();

    updateRunFunctions(ourTeam, otherTeam, centroidDef, centroidAtk, deecision.getBall());
}
void gamewindow::updateRunFunctions(vector<robot> robots, vector<robot> enemy, Point2f def, Point2f atk, dataState b)
{
    FOR_EACH_ROBOT_FUNCTIONS(setRobots(robots));
    FOR_EACH_ROBOT_FUNCTIONS(setEnemy(enemy));
    FOR_EACH_ROBOT_FUNCTIONS(setAreas(def,atk));
    FOR_EACH_ROBOT_FUNCTIONS(setBall(b));
    FOR_EACH_ROBOT_FUNCTIONS(setStrategy(STRATEGY));
    FOR_EACH_MOVER(setRobots(robots));
    FOR_EACH_MOVER(setAreas(def,atk));
    FOR_EACH_MOVER(setBall(b));

    FOR_EACH_ROBOT_FUNCTIONS(start());

    FOR_EACH_ROBOT_FUNCTIONS(wait());

    mover[0]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2]);
    mover[1]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2]);
    mover[2]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2]);

    FOR_EACH_MOVER(start());
    FOR_EACH_MOVER(wait());
}

int gamewindow::get_strategy()
{
    return STRATEGY;
}

void gamewindow::EnviaVelocidades(Point2f Velocidades[3]){

    for(int index = 0; index < 3; index ++){
        Velocidades[index].x = mover[index]->getLVel();
        Velocidades[index].y = mover[index]->getRVel();
    }
}
