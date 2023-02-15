#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <QThread>
#include "robot.h"
#include "config.h"
#define FIXED3 0
#define FULL_ATK 1
#define SAFE_FULL_ATK 2
#define NS 3
#define CINCO 4

class Navigation: public QThread
{
    Q_OBJECT
public:
    Navigation();
    void univectorField(dataState,Point2f);
    float univectorFieldForPlot(dataState,Point2f);
    void AddPlotPoint(Point2f );
    void AddAtkPoint(Point2f );
    float hyperbolicSpiral(dataState);
    float hyperbolicSpiral2(dataState);
    float GaussianFunc(float);
    float repulsiveAngle(Point2f, Point2f);
    float repulsiveMath(dataState, Point2f);
    float de, deW;
    float thetaDir;
    float gSize, gSizeW;
    float Kr, KrW;
    float thePhi;
    float alpha;
    float k_larg;
    Point2f goal;
    vector<Point2f> plotThePhi;
    vector<int> path;
    vector<Point2f> atkPaths;

    ~Navigation();
};

#endif // NAVIGATION_H
