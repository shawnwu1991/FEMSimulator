#include "stdafx.h"
#include "femsimulator.h"
#include <QtWidgets/QApplication>

#include "WinConsole.h"

int main(int argc, char *argv[])
{
    CWinConsole::CreateWinConsole();

    QApplication a(argc, argv);
    FEMSimulator w;
    w.resize(960,720);
    w.show();
    int r = a.exec();
    
    CWinConsole::FreeWinConsole();
    return r;
}
