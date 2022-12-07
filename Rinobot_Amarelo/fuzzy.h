#ifndef FUZZY_H
#define FUZZY_H
#include "Headers.h"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

class Fuzzy
{
public:
    Fuzzy();
    fl::Engine* engine = new fl::Engine;
    fl::InputVariable* bolaX = new fl::InputVariable;
    fl::InputVariable* bolaY = new fl::InputVariable;
    fl::InputVariable* roboX = new fl::InputVariable;
    fl::InputVariable* roboY = new fl::InputVariable;
    fl::InputVariable* velBola = new fl::InputVariable;

    fl::OutputVariable* fuzzyX = new fl::OutputVariable;
    fl::OutputVariable* fuzzyY = new fl::OutputVariable;
    fl::OutputVariable* troca = new fl::OutputVariable;

    fl::RuleBlock* mamdani = new fl::RuleBlock;

    void inicializaInputs();
    void inicializaOutputs();
    void inicializaRules();
    void start();
    void exemploDeTeste();
    Point3f encontraDestino(Point2f ball, Point2f strike, float velBall);
};

#endif // FUZZY_H
