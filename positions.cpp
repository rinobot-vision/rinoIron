#include "positions.h"
#include "list"

std::vector<float> Position::references;
int Position::_divisions;
Point2f Position::upperBoundPos;
Point2f Position::lowerBoundPos;
Point2f Position::lastSafePos;
Point2f Position::centroidDef;
Point2f Position::goalKeeperPos;
Point2f Position::strikerPos;
Point2f Position::centroidAtk;

void Position::Init(int divisions) {
    references.reserve(divisions);
    _divisions = divisions;
    for(int i = 0; i < divisions; i++) {
        references.push_back((INTERVAL_MAX - INTERVAL_MIN) / divisions * i + INTERVAL_MIN);
    }
    for(auto ref : references) {
        std::cout << ref << " ";
    }
    std::cout << std::endl;
    upperBoundPos.x = lowerBoundPos.x = 37;
    lastSafePos.x = INT_MIN;
    lastSafePos.y = INT_MIN;
    centroidDef.x = 10;
    centroidDef.y = 65;
    centroidAtk.y = 65;
    centroidAtk.x = 140;
    upperBoundPos.y = centroidDef.y - 42;
    lowerBoundPos.y = centroidDef.y + 42;
}

Point2f Position::getLiberoGoal(Point2f ballPos, Point2f liberoPos) {
    //CORE_WARN("GOALKEEPER X: {} and Y: {}", goalKeeperPos.x, goalKeeperPos.y);
    Point2f goal;
    int interval = (ballPos.x - INTERVAL_MIN) * _divisions / (INTERVAL_MAX - INTERVAL_MIN);
    goal.x = references[interval];
    if(ballPos.x > INTERVAL_MAX) {
        goal.x = references.back();
    }
    else if(ballPos.x < INTERVAL_MIN) {
        goal.x = references.front();
    }
    goal.y = ballPos.y;
    if(ballPos.x < centroidDef.x + 28 && ballPos.y > centroidDef.y - 35 && ballPos.y < centroidDef.y + 35) {
        goal.x = centroidDef.x + 30;
        if(ballPos.y > centroidDef.y)
        {
            goal.y = goalKeeperPos.y - 30;
        }
        else
        {
            goal.y = goalKeeperPos.y + 30;
        }
    }
    else if(ballPos.y < centroidDef.y - 35 || ballPos.y > centroidDef.y + 35) {
        goal.y = ballPos.y;
        goal.x = ballPos.x;
    }
    if(ballPos.x > centroidAtk.x - 20 && ballPos.y > centroidDef.y - 35 && ballPos.y < centroidDef.y + 35 && euclidean_dist(strikerPos, ballPos) > euclidean_dist(lastSafePos, ballPos)) {
       goal = ballPos; 
    }
    lastSafePos = goal;
    return goal;
}

bool Position::checkPosOnDefenderLine(Point2f liberoPos) {
    Point2f goal;
    int interval = (liberoPos.x - INTERVAL_MIN) * _divisions / (INTERVAL_MAX - INTERVAL_MIN);
    goal.x = references[interval];
    goal.y = liberoPos.y;
    if(euclidean_dist(goal, liberoPos) < 2.5) {
        return true;
    }
    return false;
}

void Position::setGoalKeeperPos(Point2f _goalKeeperPos) {
    goalKeeperPos = _goalKeeperPos;
}

void Position::setStrikerPos(Point2f _strikerPos) {
    strikerPos = _strikerPos;
}