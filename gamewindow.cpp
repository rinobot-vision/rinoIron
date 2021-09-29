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
    deecision.setAreas(def,atk);
    deecision.setBall(ball);
    deecision.setKnnInformation(k);
    deecision.setStrategy(STRATEGY);
    deecision.updateObjectives();
    vector<robot> teste;
    teste = deecision.getTeamRobots();
    updateRunFunctions(teste,deecision.getCentroidDef(),deecision.getCentroidAtk(),deecision.getBall());

}
void gamewindow::updateRunFunctions(vector<robot> robots, Point2f def, Point2f atk, dataState b)
{
    FOR_EACH_ROBOT_FUNCTIONS(setRobots(robots));
    FOR_EACH_ROBOT_FUNCTIONS(setAreas(def,atk));
    FOR_EACH_ROBOT_FUNCTIONS(setBall(b));
    FOR_EACH_ROBOT_FUNCTIONS(setStrategy(STRATEGY));
    FOR_EACH_MOVER(setRobots(robots));
    FOR_EACH_MOVER(setAreas(def,atk));
    FOR_EACH_MOVER(setBall(b));

    FOR_EACH_ROBOT_FUNCTIONS(start());
    FOR_EACH_ROBOT_FUNCTIONS(wait());

    mover[0]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2],robotFunctions[3],robotFunctions[4]);
    mover[1]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2],robotFunctions[3],robotFunctions[4]);
    mover[2]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2],robotFunctions[3],robotFunctions[4]);
    mover[3]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2],robotFunctions[3],robotFunctions[4]);
    mover[4]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2],robotFunctions[3],robotFunctions[4]);

    FOR_EACH_MOVER(start());
    FOR_EACH_MOVER(wait());

}

void gamewindow::EnviaVelocidades(Point2f Velocidades[5]){

    for(int index = 0; index < 5; index ++){
        Velocidades[index].x = mover[index]->getLVel();
        Velocidades[index].y = mover[index]->getRVel();
    }
}
