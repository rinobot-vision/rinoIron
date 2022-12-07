#include <QApplication>

#include "gamewindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    gamewindow w;
    w.show();
    return a.exec();
}
