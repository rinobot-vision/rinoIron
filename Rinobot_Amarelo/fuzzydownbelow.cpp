#include "fuzzydownbelow.h"
#include <string>
#include <fstream>

FuzzyDownBelow::FuzzyDownBelow()
{

}

void FuzzyDownBelow::inicializaInputs()
{
    bolaX->setName("bolaX");
    bolaX->setDescription("");
    bolaX->setEnabled(true);
    bolaX->setRange(0, 150);
    bolaX->setLockValueInRange(false);
    bolaX->addTerm(new fl::Triangle("muitoDefesa",0,0,49.04));
    bolaX->addTerm(new fl::Triangle("defesa",25.96,56.54,86.54));
    bolaX->addTerm(new fl::Triangle("ataque",63.46,93.75,124));
    bolaX->addTerm(new fl::Triangle("muitoAtaque",101,150,150));
    engine->addInputVariable(bolaX);

    bolaY->setEnabled(true);
    bolaY->setName("bolaY");
    bolaY->setDescription("");
    bolaY->setRange(0, 130);
    bolaY->setLockValueInRange(false);
    bolaY->addTerm(new fl::Triangle("muitoEsquerda",0, 0, 42.5));
    bolaY->addTerm(new fl::Triangle("esquerda",22.5, 49, 75));
    bolaY->addTerm(new fl::Triangle("direita",55 ,81.25 ,107.5));
    bolaY->addTerm(new fl::Triangle("muitoDireita",87.5,130,130));
    engine->addInputVariable(bolaY);

    roboX->setName("roboX");
    roboX->setDescription("");
    roboX->setEnabled(true);
    roboX->setRange(0, 150);
    roboX->setLockValueInRange(false);
    roboX->addTerm(new fl::Triangle("muitoDefesa",0,0,49.04));
    roboX->addTerm(new fl::Triangle("defesa",25.96,56.54,86.54));
    roboX->addTerm(new fl::Triangle("ataque",63.46,93.75,124));
    roboX->addTerm(new fl::Triangle("muitoAtaque",101,150,150));
    engine->addInputVariable(roboX);

    roboY->setEnabled(true);
    roboY->setName("roboY");
    roboY->setDescription("");
    roboY->setRange(0, 130);
    roboY->setLockValueInRange(false);
    roboY->addTerm(new fl::Triangle("muitoEsquerda",0, 0, 42.5));
    roboY->addTerm(new fl::Triangle("esquerda",22.5, 49, 75));
    roboY->addTerm(new fl::Triangle("direita",55 ,81.25 ,107.5));
    roboY->addTerm(new fl::Triangle("muitoDireita",87.5,130,130));
    engine->addInputVariable(roboY);
}

void FuzzyDownBelow::inicializaOutputs()
{
    fuzzyX->setEnabled(true);
    fuzzyX->setName("fuzzyX");
    fuzzyX->setDescription("");
    fuzzyX->setRange(0, 150);
    fuzzyX->setLockValueInRange(false);
    fuzzyX->setAggregation(new fl::Maximum);
    fuzzyX->setDefuzzifier(new fl::Centroid(200));
    fuzzyX->setDefaultValue(fl::nan);
    fuzzyX->setLockPreviousValue(false);

    fuzzyX->addTerm(new fl::Triangle("muitoDefesa",0,0,49.04));
    fuzzyX->addTerm(new fl::Triangle("defesa",25.96,56.54,86.54));
    fuzzyX->addTerm(new fl::Triangle("ataque",63.46,93.75,124));
    fuzzyX->addTerm(new fl::Triangle("muitoAtaque",101,150,150));
    engine->addOutputVariable(fuzzyX);

    fuzzyY->setEnabled(true);
    fuzzyY->setName("fuzzyY");
    fuzzyY->setDescription("");
    fuzzyY->setRange(0, 130);
    fuzzyY->setLockValueInRange(false);
    fuzzyY->setAggregation(new fl::Maximum);
    fuzzyY->setDefuzzifier(new fl::Centroid(200));
    fuzzyY->setDefaultValue(fl::nan);
    fuzzyY->setLockPreviousValue(false);

    fuzzyY->addTerm(new fl::Triangle("muitoEsquerda",0, 0, 42.5));
    fuzzyY->addTerm(new fl::Triangle("esquerda",22.5, 49, 75));
    fuzzyY->addTerm(new fl::Triangle("direita",55 ,81.25 ,107.5));
    fuzzyY->addTerm(new fl::Triangle("muitoDireita",87.5,130,130));
    engine->addOutputVariable(fuzzyY);

    troca->setName("troca");
    troca->setDescription("");
    troca->setRange(0, 1);
    troca->setLockValueInRange(false);
    troca->setAggregation(new fl::Maximum);
    troca->setDefuzzifier(new fl::Centroid(200));
    troca->setDefaultValue(fl::nan);
    troca->setLockPreviousValue(false);
    troca->addTerm(new fl::Triangle("falso", 0,0,0.5));
    troca->addTerm(new fl::Triangle("verdade",0.5,1,1));
    engine->addOutputVariable(troca);


}

void FuzzyDownBelow::inicializaRules()
{
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new fl::AlgebraicProduct);
    mamdani->setDisjunction(new fl::AlgebraicSum);
    mamdani->setImplication(new fl::Minimum);
    mamdani->setActivation(new fl::General);

    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is defesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is defesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is defesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is defesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is ataque and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is ataque and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is ataque and roboY is direita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is ataque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is esquerda then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is direita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is muitoDefesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is muitoDefesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is defesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is defesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is muitoDireita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is ataque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is ataque and roboY is esquerda then fuzzyX is defesa and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is ataque and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is ataque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is muitoAtaque and roboY is esquerda then fuzzyX is defesa and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is muitoAtaque and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is esquerda and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is muitoDefesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is muitoDefesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is muitoEsquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is defesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is defesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is ataque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is ataque and roboY is esquerda then fuzzyX is defesa and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is ataque and roboY is direita then fuzzyX is defesa and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is ataque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is muitoAtaque and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is muitoAtaque and roboY is direita then fuzzyX is defesa and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is direita and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is muitoDefesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is muitoDefesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is defesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is defesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is defesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is defesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is ataque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is ataque and roboY is esquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is ataque and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is ataque and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is muitoAtaque and roboY is esquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is muitoAtaque and roboY is direita then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoDefesa and bolaY is muitoDireita and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is defesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is defesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is defesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is ataque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is ataque and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is ataque and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is ataque and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is muitoDefesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is muitoDefesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is defesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is defesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is defesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is defesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is ataque and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is ataque and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is ataque and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is ataque and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is muitoAtaque and roboY is esquerda then fuzzyX is defesa and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is muitoAtaque and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is esquerda and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is muitoDefesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is muitoDefesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is defesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is defesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is defesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is defesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is ataque and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is ataque and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is ataque and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is ataque and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is muitoAtaque and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is muitoAtaque and roboY is direita then fuzzyX is defesa and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is direita and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is defesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is defesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is defesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is ataque and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is ataque and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is ataque and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is ataque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is muitoAtaque and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is muitoAtaque and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is defesa and bolaY is muitoDireita and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is ataque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is ataque and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is ataque and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is ataque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is muitoAtaque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is esquerda then fuzzyX is muitoAtaque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is muitoDefesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is defesa and roboY is direita then fuzzyX is muitoDefesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is ataque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is ataque and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is ataque and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is ataque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is muitoAtaque and roboY is esquerda then fuzzyX is muitoAtaque and fuzzyY is muitoEsquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is muitoAtaque and roboY is direita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is esquerda and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is muitoDefesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is defesa and roboY is esquerda then fuzzyX is muitoDefesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is ataque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is ataque and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is ataque and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is ataque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is muitoAtaque and roboY is esquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is muitoAtaque and roboY is direita then fuzzyX is muitoAtaque and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is direita and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is muitoDireita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is ataque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is ataque and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is ataque and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is ataque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is muitoAtaque and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is muitoAtaque and roboY is direita then fuzzyX is muitoAtaque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is ataque and bolaY is muitoDireita and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is muitoAtaque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is ataque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is ataque and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is ataque and roboY is direita then fuzzyX is ataque and fuzzyY is muitoEsquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is ataque and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is esquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is direita then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoEsquerda and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is ataque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is ataque and roboY is esquerda then fuzzyX is ataque and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is ataque and roboY is direita then fuzzyX is ataque and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is ataque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is muitoAtaque and roboY is esquerda then fuzzyX is ataque and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is muitoAtaque and roboY is direita then fuzzyX is ataque and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is esquerda and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is ataque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is ataque and roboY is esquerda then fuzzyX is ataque and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is ataque and roboY is direita then fuzzyX is ataque and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is ataque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is muitoAtaque and roboY is esquerda then fuzzyX is ataque and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is muitoAtaque and roboY is direita then fuzzyX is ataque and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is direita and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is muitoDefesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is muitoDefesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is muitoDefesa and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is muitoDefesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is defesa and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is defesa and roboY is esquerda then fuzzyX is defesa and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is defesa and roboY is direita then fuzzyX is defesa and fuzzyY is esquerda and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is defesa and roboY is muitoDireita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is ataque and roboY is muitoEsquerda then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is ataque and roboY is esquerda then fuzzyX is ataque and fuzzyY is muitoDireita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is ataque and roboY is direita then fuzzyX is defesa and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is ataque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is falso", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is muitoAtaque and roboY is muitoEsquerda then fuzzyX is ataque and fuzzyY is esquerda and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is muitoAtaque and roboY is esquerda then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is muitoAtaque and roboY is direita then fuzzyX is ataque and fuzzyY is direita and troca is verdade", engine));
    mamdani->addRule(fl::Rule::parse("if bolaX is muitoAtaque and bolaY is muitoDireita and roboX is muitoAtaque and roboY is muitoDireita then fuzzyX is ataque and fuzzyY is direita and troca is falso", engine));
    engine->addRuleBlock(mamdani);
}

void FuzzyDownBelow::start()
{
    engine->setName("encontraPosicao");
    engine->setDescription("");
    inicializaInputs();
    inicializaOutputs();
    inicializaRules();
}

void FuzzyDownBelow::exemploDeTeste()
{
    std::string status;
    if (not engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);

    fl::scalar bolX = 120;
    fl::scalar bolY = 30;
    bolaX->setValue(bolX);
    bolaY->setValue(bolY);
    fl::scalar robX = 10;
    fl::scalar robY = 20;
    roboX->setValue(robX);
    roboY->setValue(robY);
    //fl::scalar veloBall = 0;
    //velBola->setValue(veloBall);

    engine->process();
    cout <<"(" <<fuzzyX->getValue()<< ", " << fuzzyY->getValue() << ")" << std::endl;
    cout << "valor troca: " << troca->getValue() << endl;
}

Point2f FuzzyDownBelow::encontraPosicao(Point2f ball, Point2f strike) //float velBall
{
    std::string status;
    if (not engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);

    fl::scalar bolX = ball.x;
    fl::scalar bolY = ball.y;
    bolaX->setValue(bolX);
    bolaY->setValue(bolY);
    fl::scalar robX = strike.x;
    fl::scalar robY = strike.y;
    roboX->setValue(robX);
    roboY->setValue(robY);
    //fl::scalar veloBall = velBall;
    //velBola->setValue(veloBall);

    engine->process();

    return Point2f(fuzzyX->getValue(), fuzzyY->getValue());
}

float FuzzyDownBelow::decideTroca(Point2f ball, Point2f strike) //float velBall
{
    std::string status;
    if (not engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);

    fl::scalar bolX = ball.x;
    fl::scalar bolY = ball.y;
    bolaX->setValue(bolX);
    bolaY->setValue(bolY);
    fl::scalar robX = strike.x;
    fl::scalar robY = strike.y;
    roboX->setValue(robX);
    roboY->setValue(robY);
    //fl::scalar veloBall = velBall;
    //velBola->setValue(veloBall);

    engine->process();

    float varTroca = troca->getValue();
    return varTroca;
}
