#include "positions.h"
#include "list"

std::vector<float> Position::references;
int Position::_divisions;
Point2f Position::upperBoundPos;
Point2f Position::lowerBoundPos;
Point2f Position::lastSafePos;
Point2f Position::centroidDef;

void Position::Init(int divisions) {
    references.reserve(divisions);
    _divisions = divisions;
    for(int i = 0; i < divisions; i++) {
        references[i] = (INTERVAL_MAX - INTERVAL_MIN) / divisions * i + INTERVAL_MIN;
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
    upperBoundPos.y = centroidDef.y + 42;
    lowerBoundPos.y = centroidDef.y - 42;
}

Point2f Position::getLiberoGoal(Point2f ballPos, Point2f liberoPos) {
    Point2f goal;
    int interval = (ballPos.x - INTERVAL_MIN) * _divisions / (INTERVAL_MAX - INTERVAL_MIN);
    if(ballPos.x > INTERVAL_MAX) return lastSafePos;
    if(ballPos.x < 30) {
        if(ballPos.y <= INTERVAL_MIN) {
            return upperBoundPos;
        }
        else {
            return lowerBoundPos;
        }
    }
    if(interval >= 9 || interval <= 0) {
        return lastSafePos;
    }
    goal.x = references[interval];
    goal.y = ballPos.y;
    lastSafePos = goal;
    return goal;
}