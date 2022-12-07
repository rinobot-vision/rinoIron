#ifndef KNN_H
#define KNN_H
#include "robot.h"
#define SIT 4380

struct Situation
{
    int quadrant[4];            // vetor que armazena em qual quadrante cada robô e a bola estão: [ robo1, robo2, robo3, bola]
    int robotFunction[3];       // função de jogo
    int situationNumber;	// salva o indice da situação

    Situation()
    {
    quadrant[0] = quadrant[1] = quadrant[2] = quadrant[3] = 0;
    robotFunction[0] = robotFunction[1] = robotFunction[2] = 0;
    situationNumber = 0;
    };
    Situation(int a[4], int b[3], int c)
    {
    this->quadrant[0] = a[0];
    this->quadrant[1] = a[1];
    this->quadrant[2] = a[2];
    this->quadrant[3] = a[3];
    this->robotFunction[0] = b[0];
    this->robotFunction[1] = b[1];
    this->robotFunction[2] = b[2];
    this->situationNumber = c;
    };
    Situation(Situation *b)
    {
    quadrant[0] = b->quadrant[0];
    quadrant[1] = b->quadrant[1];
    quadrant[2] = b->quadrant[2];
    quadrant[3] = b->quadrant[3];
    robotFunction[0] = b->robotFunction[0];
    robotFunction[1] = b->robotFunction[1];
    robotFunction[2] = b->robotFunction[2];
    situationNumber = b->situationNumber;
    };
    void show()
    {
    cout<< "Quadrant: "<< quadrant<<endl;
    cout<<"robotFunction: "<< robotFunction<<endl;
    cout<<"situationNumber :"<< situationNumber<<endl;
    };
};

class KNN
{
public:
    KNN();
    int return_int(char a);
    int returnQuadrant(float positionX, float positionY);
    void load_database();
    void loadFunctions(vector<robot> &teamRobot, dataState ball);
    Situation sit[SIT];
};


#endif // KNN_H
