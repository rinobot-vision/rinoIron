#ifndef FUZZYDOWNBELOW_H
#define FUZZYDOWNBELOW_H

#include "fl/Headers.h"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

class FuzzyDownBelow
{
public:
    FuzzyDownBelow();
    fl::Engine* engine = new fl::Engine;
    fl::InputVariable* bolaX = new fl::InputVariable;
    fl::InputVariable* bolaY = new fl::InputVariable;
    fl::InputVariable* roboX = new fl::InputVariable;
    fl::InputVariable* roboY = new fl::InputVariable;
    //fl::InputVariable* velBola = new fl::InputVariable;

    fl::OutputVariable* fuzzyX = new fl::OutputVariable;
    fl::OutputVariable* fuzzyY = new fl::OutputVariable;
    fl::OutputVariable* troca = new fl::OutputVariable;

    fl::RuleBlock* mamdani = new fl::RuleBlock;



    void inicializaInputs();
    void inicializaOutputs();
    void inicializaRules();
    void start();
    void exemploDeTeste();
    Point2f encontraPosicao(Point2f, Point2f); //float
    float decideTroca(Point2f, Point2f); //float

};

#endif // FUZZYDOWNBELOW_H
