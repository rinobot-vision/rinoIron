
#ifndef GAMEFUNCTIONS_H
#define GAMEFUNCTIONS_H

#include "navigation.h"
#include "robot.h"
#include "fuzzy.h"
#include "fuzzytorneio.h"
#include "fuzzydownbelow.h"


class GameFunctions: public Navigation
{
public:
    GameFunctions();
    void setRobots(vector<robot>);
    void setAreas(Point2d, Point2d);
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
    bool airball;
    void setStopOnGoal(bool);
    clock_t speedmf;
    bool fedUp;
    bool getStopOnGoal();
    void striker();
    void defender();
    void goalkeeper();
    void fake9();
    void midfield();
    void wing();
    void volante();
    void fuzzy();
    void impostor();
    void libero();
    void newstriker();


    void PlotPath(int i, robot);
    void AtkPath(robot);
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
    Point2d getGoal();
    float getDirection();
    Point2d atkPoint;
    ~GameFunctions();
    int getStrategy();
    void setStrategy(int);

    vector<robot> teamRobot;
    Fuzzy objetoFuzzy;
    Fuzzy_Torneio fuzzytorneio;
    FuzzyDownBelow fuzzydownbelow;

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
    int indexRobot;
    Point2d centroidDef, centroidAtk;
    dataState ball;

    bool tiroMetaSituation = false;
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
    bool atkSituationInv = false;
    bool againstTheTeam = false;
    bool plotState = false;
    bool kickPState = false;
    bool flagGoAhead = false;
    bool flagKickBall = false;
    bool penaultyPermission = false;
    float defenderLine = 38;
    float volanteLine = 38;
    int strategy;
    float lastX = defenderLine;


};

#endif // GAMEFUNCTIONS_H
