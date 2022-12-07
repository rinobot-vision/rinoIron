#include "gamewindow.h"
#include "ui_gamewindow.h"

#define FOR_EACH_ROBOT_FUNCTIONS(methodcall)           \
    do{                                                \
    for(int i = 0; i < NUMROBOTS; i++){            \
    robotFunctions[i]->methodcall;             \
    }                                              \
    }while(0);

#define FOR_EACH_MOVER(methodcall)                     \
    do{                                                \
    for(int i = 0; i < NUMROBOTS; i++){            \
    mover[i]->methodcall;                      \
    }                                              \
    }while(0);


gamewindow::gamewindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::gamewindow)
{
    ui->setupUi(this);

    qRegisterMetaType<vector<robot>>("vector<robot>");
    qRegisterMetaType<Point2d>("Point2d");
    qRegisterMetaType<dataState>("dataState");
    qRegisterMetaType<KNN>("KNN");

    for(int i = 0; i < NUMROBOTS; i++)
    {
        robotFunctions[i] = new GameFunctions();
        mover[i] = new Mover();
    }

    for(int i = 0; i < NUMROBOTS; i++){
        mover[i]->setIndex(i);
        robotFunctions[i]->setIndex(i);
    }
    ui->KpSpinBox->setValue(mover[0]->getKp());
    ui->KdSpinBox->setValue(mover[0]->getKd());
    ui->gSizeWSpinBox->setValue(robotFunctions[0]->getgSizeW());
    ui->deWSpinBox->setValue(robotFunctions[0]->getdeW());
    ui->krWSpinBox->setValue(robotFunctions[0]->getkrW());
    ui->kLargSpinBox->setValue(robotFunctions[0]->getkLarg());
    ui->defenderLineSpinBox->setValue(robotFunctions[0]->getDefenderLine());
    ui->StrategycomboBox->setCurrentIndex(STRATEGY);
    actuatorClient->setTeamColor(ourColor);
    connect(vision,SIGNAL(emit_info(vector<robot>,vector<robot>,Point2d,Point2d,dataState,KNN)),this ,SLOT(updateInfo(vector<robot>,vector<robot>,Point2d,Point2d,dataState,KNN)),Qt::QueuedConnection);
    vision->start();
}


gamewindow::~gamewindow()
{
    delete ui;
}

void gamewindow::updateInfo(vector<robot> robots, vector<robot> enemy, Point2d def, Point2d atk, dataState ball, KNN k)
{
    deecision.setRobots(robots);
    deecision.setAreas(def,atk);
    deecision.setBall(ball);
    deecision.setKnnInformation(k);
    deecision.setStrategy(STRATEGY);
    deecision.updateObjectives();
    vector<robot> teste;
    teste = deecision.getTeamRobots();
    updateRunFunctions(teste,deecision.getCentroidDef(),deecision.getCentroidAtk(),deecision.getBall());

}
void gamewindow::updateRunFunctions(vector<robot> robots, Point2d def, Point2d atk, dataState b)
{
    FOR_EACH_ROBOT_FUNCTIONS(setRobots(robots));
    FOR_EACH_ROBOT_FUNCTIONS(setAreas(def,atk));
    FOR_EACH_ROBOT_FUNCTIONS(setBall(b));
    FOR_EACH_ROBOT_FUNCTIONS(setStrategy(STRATEGY));
    FOR_EACH_MOVER(setRobots(robots));
    FOR_EACH_MOVER(setAreas(def,atk));
    FOR_EACH_MOVER(setBall(b));

    FOR_EACH_ROBOT_FUNCTIONS(start());
    FOR_EACH_ROBOT_FUNCTIONS(wait());

    mover[0]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2]);
    mover[1]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2]);
    mover[2]->setGameFunctions(robotFunctions[0],robotFunctions[1],robotFunctions[2]);

    FOR_EACH_MOVER(start());
    FOR_EACH_MOVER(wait());

    if(startButton == true){
        for(int i = 0; i < 3; i++){
            actuatorClient->sendCommand(i, mover[i]->getLVel(), mover[i]->getRVel());
        }
    }
    else{
        for(int i = 0; i < 3; i++){
            actuatorClient->sendCommand(i, 0, 0);
        }
    }
}


void gamewindow::on_jogar_clicked()
{
    if(startButton == true)
        startButton = false;
    else
        startButton = true;
}

void gamewindow::on_KpSpinBox_valueChanged(double arg1)
{
    for(int i = 0; i < 3; i++){
        mover[i]->setKp(arg1);
    }
}

void gamewindow::on_KdSpinBox_valueChanged(double arg1)
{
    for(int i = 0; i < 3; i++){
        mover[i]->setKd(arg1);
    }
}

void gamewindow::on_gSizeWSpinBox_valueChanged(double arg1)
{
    for(int i = 0; i < 3; i++){
        robotFunctions[i]->setgSizeW(arg1);
    }
}

void gamewindow::on_deWSpinBox_valueChanged(double arg1)
{
    for(int i = 0; i < 3; i++){
        robotFunctions[i]->setdeW(arg1);
    }
}

void gamewindow::on_krWSpinBox_valueChanged(double arg1)
{
    for(int i = 0; i < 3; i++){
        robotFunctions[i]->setkrW(arg1);
    }
}

void gamewindow::on_kLargSpinBox_valueChanged(double arg1)
{
    for(int i = 0; i < 3; i++){
        robotFunctions[i]->setkLarg(arg1);
    }
}

void gamewindow::on_defenderLineSpinBox_valueChanged(double arg1)
{
    for(int i = 0; i < 3; i++){
        robotFunctions[i]->setDefenderLine(arg1);
    }
}

void gamewindow::on_StrategycomboBox_currentIndexChanged(int index)
{
    STRATEGY = index;
}
