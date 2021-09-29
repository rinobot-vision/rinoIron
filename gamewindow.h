#ifndef GAMEWINDOW_H
#define GAMEWINDOW_H
#include "utils.h"
#include "decision.h"
#include "gamefunctions.h"
#include "knn.h"
#include "mover.h"

class gamewindow
{
public:
    gamewindow();

    vector<Point2f> posBall, posRobot0, posRobot1, posRobot2;
    Decision deecision;

    void updateInfo(vector<robot>, vector<robot>, Point2f, Point2f, dataState, KNN k);
    void updateRunFunctions(vector<robot>, Point2f, Point2f, dataState);
    GameFunctions *robotFunctions[5];
    Mover *mover[5];

    void EnviaVelocidades(Point2f Velocidades[5]);
private:
};

#endif // GAMEWINDOW_H
