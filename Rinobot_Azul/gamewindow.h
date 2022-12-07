#ifndef GAMEWINDOW_H
#define GAMEWINDOW_H
#include "utils.h"
#include "decision.h"
#include "gamefunctions.h"
#include "knn.h"
#include "mover.h"
#include <QMainWindow>
#include <clients/actuator/actuatorclient.h>
#include "vision.h"

//  Para escolher qual strategy utilizar altere os valores acima de acordo com a tabela a seguir
//  0 = FIXED
//  1 = FULL_ATK
//  2 = SAFE_FULL_ATK
//  3 = NS
//  4 = FUZZY
//  5 = KNN
//  6 = IMPOSTOR_
//  7 = DOWNBELOW
//  8 = FUZZYLEIRO


QT_BEGIN_NAMESPACE
namespace Ui { class gamewindow; }
QT_END_NAMESPACE

class gamewindow : public QMainWindow
{
    Q_OBJECT

public:
    gamewindow(QWidget *parent = nullptr);
    ~gamewindow();
    vector<Point2d> posBall, posRobot0, posRobot1, posRobot2;
    Decision deecision;

    Vision *vision = new Vision();

    void updateRunFunctions(vector<robot>, Point2d, Point2d, dataState);
    GameFunctions *robotFunctions[3];
    Mover *mover[3];
    bool startButton = false;
    void EnviaVelocidades(Point2f Velocidades[3]);
    ActuatorClient *actuatorClient = new ActuatorClient("127.0.0.1", 20011);
    VSSRef::Color ourColor = VSSRef::Color::BLUE;

    int STRATEGY = 0;

public slots:

   void updateInfo(vector<robot>, vector<robot>, Point2d, Point2d, dataState, KNN k);


private slots:

   void on_jogar_clicked();

   void on_KpSpinBox_valueChanged(double arg1);

   void on_KdSpinBox_valueChanged(double arg1);

   void on_gSizeWSpinBox_valueChanged(double arg1);

   void on_deWSpinBox_valueChanged(double arg1);

   void on_krWSpinBox_valueChanged(double arg1);

   void on_kLargSpinBox_valueChanged(double arg1);

   void on_defenderLineSpinBox_valueChanged(double arg1);

   void on_StrategycomboBox_currentIndexChanged(int index);

private:
    Ui::gamewindow *ui;
};
#endif // GAMEWINDOW_H
