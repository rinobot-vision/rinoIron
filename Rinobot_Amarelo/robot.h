#ifndef ROBOT_H
#define ROBOT_H

#include "utils.h"

#define GOALKEEPER 0
#define DEFENDER 1
#define STRIKER 2
#define FAKE9 3
#define MIDFIELD 4
#define WING 5
#define VOLANTE 6
#define FUZZYROBOT 7
#define STRIKERAGAINSTEAM 8
#define IMPOSTOR 9
#define LIBERO 13
#define NEWSTRIKER 12

class robot
{
public:
    robot();
    void setKp(double);
    void setKd(double);
    void setKi(double);
    void setFunction(int);
    double getKp();
    double getKd();
    double getKi();
    dataState getDataState();
    int getFunction();
    void setPosition(Point2f pos, double ang, Point2d vel);
    void setQuadrant(int quadrant);
    vector<Point2f> pathPoints;
    void cleanPath();
    void AddPathPoint(Point2f);
    void setTime(int);
    int getTime();
    Point2f getPosition();
    double getAngle();
    Point2d getVelocidade();


private:
    dataState kinetic;
    int function;
    double kp;
    double kd;
    double ki;
    int index;
    int time;
};

#endif // ROBOT_H
