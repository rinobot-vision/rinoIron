#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <time.h>
#include "math.h"
#include "iostream"
#include "fstream"

#include <eigen3/Eigen/Dense>

#include <QDebug>
#include <unistd.h>

#include <QDateTime>
#include <QElapsedTimer>
#include <QDir>

#include "QObject"


#define CLEAR(x) memset(&(x), 0, sizeof(x))


// se definido como 1 o programa calcula e mostra o tempo de execução

#define PI 3.141592
#define NUMROBOTS 3
#define FPS 100
#define STRATEGY 1

//  Para escolher qual strategy utilizar altere os valores acima de acordo com a tabela a seguir
//  0 = FIXED
//  1 = FULL_ATK
//  2 = SAFE_FULL_ATK
//  3 = NS
//  5 = KNN
//  6 - OFF

struct Point2f{
    float x,y;
    Point2f(float x, float y){
        this ->x = x;
        this ->y = y;
    }
    Point2f(){
        x = 0;
        y = 0;
    }
};



using namespace std;
using namespace Eigen;

struct dataState
{
    Point2f pos;
    Point2f vel;
    double angle;
    double omega;
    int quadrant;
};

vector<Point2f> leastSquares(vector<Point2f>);
vector<double> leastSquares(vector<double>);
double angleTwoPoints(Point2f p, Point2f q);
double euclidean_dist(Point2f p, Point2f q);
double ajustaAngulo(double);



#endif // UTILS_H
