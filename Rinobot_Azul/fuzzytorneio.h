#ifndef FUZZYTORNEIO_H
#define FUZZYTORNEIO_H
#include "Headers.h"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

class Fuzzy_Torneio
{
public:
    Fuzzy_Torneio();

    fl::Engine* engine = new fl::Engine;
    fl::InputVariable* ballX = new fl::InputVariable;
    fl::InputVariable* ballY = new fl::InputVariable;
    fl::InputVariable* strikerX = new fl::InputVariable;
    fl::InputVariable* strikerY = new fl::InputVariable;
    fl::InputVariable* liberoX = new fl::InputVariable;
    fl::InputVariable* liberoY = new fl::InputVariable;
    fl::InputVariable* ballspeed = new fl::InputVariable;

    fl::OutputVariable* strikergoalX = new fl::OutputVariable;
    fl::OutputVariable* strikergoalY = new fl::OutputVariable;
    fl::OutputVariable* change = new fl::OutputVariable;

    fl::RuleBlock* mamdani = new fl::RuleBlock;

    void inicializaInputs();
    void inicializaOutputs();
    void inicializaRules();
    void start();
    void exemploDeTeste();
    Point3f encontraDestino(Point2f ball, Point2f strike, Point2f libero);
};

#endif // FUZZY_H
