#include "log.h"
#include "utils.h"
#include "memory"
#include "vector"
#include "iostream"
#include "config.h"
#include "climits"


#ifndef POSITION_H
#define POSITION_H

#define INTERVAL_MAX 140
#define INTERVAL_MIN 40

class Position {
    public:
        static Point2f getLiberoGoal(Point2f ballPos, Point2f liberoPos);
        static void Init(int divisions);
        static bool checkPosOnDefenderLine(Point2f liberoPos);
        static void setGoalKeeperPos(Point2f _goalKeeperPos);
        static void setStrikerPos(Point2f _strikerPos);
    private:
        static std::vector<float> references;
        static int _divisions;
        static Point2f lastSafePos;
        static Point2f upperBoundPos;
        static Point2f lowerBoundPos;
        static Point2f centroidDef;
        static Point2f centroidAtk;
        static Point2f strikerPos;
        static Point2f goalKeeperPos;
};

#endif