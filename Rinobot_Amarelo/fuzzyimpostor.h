#ifndef FUZZYIMPOSTOR_H
#define FUZZYIMPOSTOR_H

#include "Headers.h"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

class fuzzyImpostor
{
public:
    fuzzyImpostor();
    fl::Engine* engine = new fl::Engine;
    fl::InputVariable* RoboAX = new fl::InputVariable;
    fl::InputVariable* RoboAY = new fl::InputVariable;
    fl::InputVariable* RoboBX = new fl::InputVariable;
    fl::InputVariable* RoboBY = new fl::InputVariable;
    fl::InputVariable* RoboCX = new fl::InputVariable;
    fl::InputVariable* RoboCY = new fl::InputVariable;
    fl::InputVariable* BolaX = new fl::InputVariable;
    fl::InputVariable* BolaY = new fl::InputVariable;

    fl::OutputVariable* RoboAF = new fl::OutputVariable;
    fl::OutputVariable* RoboBF = new fl::OutputVariable;
    fl::OutputVariable* Robo3F = new fl::OutputVariable;

    fl::RuleBlock* mamdani = new fl::RuleBlock;



    void inicializaInputs();
    void inicializaOutputs();
    void inicializaRules();
    void start();
    void exemploDeTeste();
    Point3f encontraDestino(Point2f, Point2f, Point2f, Point2f);
};

#endif // FUZZYIMPOSTOR_H
