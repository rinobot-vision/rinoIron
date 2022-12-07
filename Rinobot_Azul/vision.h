#ifndef VISION_H
#define VISION_H
#include "utils.h"
#include "robot.h"
#include <QThread>

#include <thread>
#include <utils/timer/timer.h>
#include <clients/vision/visionclient.h>
#include <clients/referee/refereeclient.h>
#include <clients/actuator/actuatorclient.h>
#include <clients/replacer/replacerclient.h>
#include <stdio.h>
#include "utils.h"
#include "knn.h"
#include "QThread"

class Vision : public QThread
{
    Q_OBJECT
public:
    Vision();
    ~Vision();
    void run();
    vector<robot> robots;
    vector<robot> enemy;             //Vector para robos aliados
    //Vector para robos aliados
    dataState ball;
    Point2d centroidDef;
    Point2d centroidAtk;
    KNN Knn;

    void setCentroidDef(Point2d def);
    void setCentroidAtk(Point2d atk);
    void setRobots(vector<robot>);
    void setBall(dataState);

    vector<robot> getRobots();
    vector<robot> getEnemy();
    Point2d getCentroidDef();
    Point2d getCentroidAtk();
    dataState getBall();
    void RodaCodigo();


signals:
    void emit_info(vector<robot>,vector<robot>,Point2d,Point2d,dataState, KNN);

};

#endif // VISION_H
