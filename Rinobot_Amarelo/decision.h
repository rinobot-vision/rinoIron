#ifndef DECISION_H
#define DECISION_H

#include <QThread>
#include "robot.h"
#include <QTimer>
#include "utils.h"
#include "knn.h"
#include "fuzzy.h"
#include "fuzzyimpostor.h"
#include "fuzzydownbelow.h"
#include "fuzzytorneio.h"

#define FIXED3 0
#define FULL_ATK 1
#define SAFE_FULL_ATK 2
#define NS 3
#define FUZZY_ 4
#define KNN_ 5
#define IMPOSTOR_ 6
#define DOWNBELOW 7
#define FUZZYT 8

#define SIT 4380

class Decision
{
public:
    explicit Decision();
    ~Decision();
    void setRobots(vector<robot>);
    void setAreas(Point2d, Point2d);
    void setBall(dataState);
    void setKnnInformation(KNN);
    void updateObjectives();
    void setStrategy(int);
    int getStrategy();
    vector<Point2f> pathPoints[3];
    clock_t clockChange;
    vector<robot> getTeamRobots();
    Point2d getCentroidDef();
    Point2d getCentroidAtk();
    dataState getBall();

    Fuzzy fuzzy;
    fuzzyImpostor FuzzyImpostor;
    FuzzyDownBelow fuzzydownbelow;
    Fuzzy_Torneio fuzzy_torneio;


private:
    vector<robot> teamRobot;
    float tempT;
    Point2d centroidDef, centroidAtk;
    dataState ball;
    bool changePermission;
    float timeChange;
    bool firstTime;
    int path1, path2;
    int strategy;
    bool swapRoles = true;
    bool flagCrossing = false;
    bool lastB = false;
    QTimer *timer1;
    clock_t clockInvert, clockTroca;
    float tempoTroca;
    clock_t clockStart, clockTrocaGS, clockTrocaFS;
    float temp, timeTrocaGS, timeTrocaFS;
    bool flagTrocaGS = true;
    bool flagTrocaFS = true;
    float LineD = 48;
    int G, D, S, F, L, M, lastG, lastD, lastS, lastM, lastL, lastF, lastW;
    int lastCompanheiroFuzzy = 1;
    int strikerIndex = 1;
    int liberoIndex = 2;
    KNN knnfunction;
};

#endif // DECISION_H
