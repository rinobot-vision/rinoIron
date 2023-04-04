#ifndef VISION_H
#define VISION_H
#include "utils.h"
#include "robot.h"
#include "gamewindow.h"
#include <QThread>

class vision : public QThread
{
    Q_OBJECT
public:
    vision();
    ~vision();
    void run();
    vector<robot> robots;
    vector<robot> enemy;             //Vector para robos aliados
    //Vector para robos aliados
    dataState ball;
    Point2f centroidDef;
    Point2f centroidAtk;

    void setCentroidDef(Point2f def);
    void setCentroidAtk(Point2f atk);
    void setRobots(vector<robot>);
    void setEnemy(vector<robot>);
    void setBall(dataState);

    vector<robot> getRobots();
    vector<robot> getEnemy();
    Point2f getCentroidDef();
    Point2f getCentroidAtk();
    dataState getBall();



signals:
    void emit_info(vector<robot>,vector<robot>,Point2f,Point2f,dataState);

};

#endif // VISION_H
