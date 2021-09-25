#ifndef MOVER_H
#define MOVER_H

#include <QMutex>
#include <QThread>
#include "gamefunctions.h"

class Mover: public QThread
{
    Q_OBJECT
public:
    Mover();
    void setVMax(float);
    float getVMax();
    float getLVel();
    float getRVel();
    void goalkeeper();
    void defender();
    void striker();
    void fake9();
    void midfield();
    void wing();
    void volante();
    void fuzzy();
    void impostor();
    void libero();
    void newstriker();
    void offdefender();
    void setGameFunctions(GameFunctions *, GameFunctions *, GameFunctions *);
    void setRobots(vector<robot>);
    void setAreas(Point2f, Point2f);
    void setAgainstTheTeam(bool);
    void setDirection(bool);
    void setBall(dataState);
    void setGains(MatrixXd);
    MatrixXd getGains();
    void rotate();
    void rotateInv();
    void kickRotate();
    void updateGains();
    void atkSituation();
    void atkSituationInv();
    float twiddle();
    void rotate_def();

    float vMax = 70;
    float kp;
    float kd;
    float temp = 0;
    float tempoTroca = 0;

    Point2f posTemp;
    Point2f prevGoal;

    bool inverte=false;
    bool sentido = false;

    clock_t clockInvert, clockTroca, clockAceleration;
    clock_t clockStart;


    void setStrategy(int);
    int getStrategy();
    void setIndex(int);
    float getKp();
    void setKp(float a);
    float getKd();
    void setKd(float a);


    ~Mover();
protected:
    void run();
private:
    GameFunctions *robotFunctions[NUMROBOTS];
    QMutex mutex;
    vector<robot> teamRobot;
    Point2f lastRobot;
    Point2f centroidDef, centroidAtk;
    dataState ball;
    MatrixXd robotGains;

    float l = 0.04;
    float alpha;
    float lastAlpha;
    float lastVel;
    float alphaS;
    float offLine = 115;
    float distGiro = 9;
    float distGiroGoleiro = 8;
    float velGiroLado = 0.8;
    float velGiroAtk = 0.5;
    float velGiroGol = 1;
    float velGiroPenalty = 2;
    float lVel;
    float rVel;

    int indexRobot;
    int robotObstCont = 0;
    int robotTimeCont = 0;
    int strategy;

    bool robotDirection = true;
    bool againstTheTeam = false;
    bool firstAceleration = true;
    bool airball;



};

#endif // MOVER_H
