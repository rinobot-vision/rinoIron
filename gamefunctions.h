
#ifndef GAMEFUNCTIONS_H
#define GAMEFUNCTIONS_H

#include "navigation.h"
#include "robot.h"

class GameFunctions: public Navigation
{
public:
    GameFunctions();
    void setRobots(vector<robot>);
    void setAreas(Point2f, Point2f);
    void setAgainstTheTeam(bool);
    bool getAgainsTheTeam();
    void setBall(dataState);
    void setAtkSituation(bool);
    void setAtkSituationTiro(bool);
    void setCrossing(bool);
    void setAtkSituationInv(bool);
    void setDefenderLine(float);
    void setVolanteLine(float);
    float getDefenderLine();
    float getVolanteLine();
    bool getAtkSituation();
    bool getAtkSituationTiro();
    bool getCrossing();
    bool getAtkSituationInv();
    void setStopOnGoal(bool);
    bool getStopOnGoal();
    void striker();
    void defender();
    void goalkeeper();
    void fake9();
    void midfield();
    void wing();
    void volante();
    void libero();
    void newstriker();
    void offdefender();
    void PlotPath(int i, robot);
    void setPlot(bool );
    bool getPlot();
    void setKickState(bool );
    bool getKickState();
    void setPenalty(bool );
    void kickBall();
    bool getPenalty();
    void setFreeBall(bool );
    bool getFreeball();
    void setTiroMeta(bool);
    bool getTiroMeta();
    void setlittleChute(bool);
    bool getlittleChute();
    void setIndex(int);
    Point2f getGoal();
    float getDirection();
    ~GameFunctions();
    int getStrategy();
    void setStrategy(int);

    bool airball;
    bool fedUp;

    Point2f atkPoint;
    clock_t speedmf;
    vector<robot> teamRobot;

    float getgSizeW();
    void setgSizeW(float a);
    float getdeW();
    void setdeW(float a);
    float getkrW();
    void setkrW(float a);
    float getkLarg();
    void setkLarg(float a);


protected:
    void run();

private:
    bool littleChuteSituation = false;
    bool freeBallSituation = false;
    bool penaltiSituation = false;
    bool atkSituation = false;
    bool atkSituationTiro = false;
    bool flagAvoidGoalkepper = false;
    bool flagCrossing = false;
    bool flagAvoidDefender = false;
    bool flagAvoidBall = false;
    bool flagStopOnGoal = false;
    bool flagMidWaiting = false;
    bool atkSituationInv = false;
    bool againstTheTeam = false;
    bool plotState = false;
    bool kickPState = false;
    bool flagGoAhead = false;
    bool flagKickBall = false;
    bool flagRepulsiveBall = false;
    bool flagRepulsiveStriker = false;
    bool penaultyPermission = false;
    bool flagGrab = false;
    bool flagGrabM = false;

    float defenderLine = 36;
    float volanteLine = 38;
    float lastX = defenderLine;
    float offLine = 115;

    int strategy;
    int indexRobot;

    dataState ball;
    Point2f StrikeRepulsiveM;
    Point2f StrikeRepulsive;

    Point2f centroidDef, centroidAtk;

    double tempoRepulsive = 0;
    double tempoStopRepulsive = 0;
    double tempoRepulsiveM = 0;
    double tempoStopRepulsiveM = 0;

    clock_t ClockStartRM;
    clock_t ClockStopRM;
    clock_t ClockStartR;
    clock_t ClockStopR;

};

#endif // GAMEFUNCTIONS_H
