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

#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <eigen3/Eigen/Dense>

#include <QDebug>
#include <unistd.h>

#include <QDateTime>
#include <QElapsedTimer>
#include <QDir>
#include <QApplication>


#include "QObject"


#define CLEAR(x) memset(&(x), 0, sizeof(x))


// se definido como 1 o programa calcula e mostra o tempo de execução

#define PI 3.141592
#define NUMROBOTS 3
#define FPS 100


using namespace std;
using namespace cv;
using namespace Eigen;

struct dataState
{
    Point2d pos;
    Point2d vel;
    double angle;
    double omega;
    int quadrant;
};

vector<Point2d> leastSquares(vector<Point2d>);
vector<Point2f> leastSquares(vector<Point2f>);
vector<double> leastSquares(vector<double>);
double angleTwoPoints(Point2d p, Point2d q);
double euclidean_dist(Point2d p, Point2d q);
double ajustaAngulo(double);



#endif // UTILS_H
