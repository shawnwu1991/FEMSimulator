/********************************************************************************
** Form generated from reading UI file 'femsimulator.ui'
**
** Created by: Qt User Interface Compiler version 5.3.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FEMSIMULATOR_H
#define UI_FEMSIMULATOR_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FEMSimulator
{
public:
    QAction *actionE_xit;
    QAction *action_Test;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menu_File;
    QMenu *menu_Test;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QToolBar *restShapeToolBar;
    QToolBar *tetMeshStat;

    void setupUi(QMainWindow *FEMSimulator)
    {
        if (FEMSimulator->objectName().isEmpty())
            FEMSimulator->setObjectName(QStringLiteral("FEMSimulator"));
        FEMSimulator->resize(600, 400);
        FEMSimulator->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
        actionE_xit = new QAction(FEMSimulator);
        actionE_xit->setObjectName(QStringLiteral("actionE_xit"));
        action_Test = new QAction(FEMSimulator);
        action_Test->setObjectName(QStringLiteral("action_Test"));
        centralWidget = new QWidget(FEMSimulator);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        FEMSimulator->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(FEMSimulator);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 23));
        menu_File = new QMenu(menuBar);
        menu_File->setObjectName(QStringLiteral("menu_File"));
        menu_Test = new QMenu(menuBar);
        menu_Test->setObjectName(QStringLiteral("menu_Test"));
        FEMSimulator->setMenuBar(menuBar);
        mainToolBar = new QToolBar(FEMSimulator);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        mainToolBar->setMinimumSize(QSize(0, 0));
        mainToolBar->setAllowedAreas(Qt::BottomToolBarArea|Qt::TopToolBarArea);
        mainToolBar->setIconSize(QSize(48, 48));
        FEMSimulator->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(FEMSimulator);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        statusBar->setStyleSheet(QStringLiteral("background-color: rgb(255, 255, 127);"));
        FEMSimulator->setStatusBar(statusBar);
        restShapeToolBar = new QToolBar(FEMSimulator);
        restShapeToolBar->setObjectName(QStringLiteral("restShapeToolBar"));
        FEMSimulator->addToolBar(Qt::RightToolBarArea, restShapeToolBar);
        tetMeshStat = new QToolBar(FEMSimulator);
        tetMeshStat->setObjectName(QStringLiteral("tetMeshStat"));
        FEMSimulator->addToolBar(Qt::TopToolBarArea, tetMeshStat);
        FEMSimulator->insertToolBarBreak(tetMeshStat);

        menuBar->addAction(menu_File->menuAction());
        menuBar->addAction(menu_Test->menuAction());
        menu_File->addAction(actionE_xit);
        menu_Test->addAction(action_Test);

        retranslateUi(FEMSimulator);

        QMetaObject::connectSlotsByName(FEMSimulator);
    } // setupUi

    void retranslateUi(QMainWindow *FEMSimulator)
    {
        FEMSimulator->setWindowTitle(QApplication::translate("FEMSimulator", "FEMSimulator", 0));
        actionE_xit->setText(QApplication::translate("FEMSimulator", "E&xit", 0));
        action_Test->setText(QApplication::translate("FEMSimulator", "&Test", 0));
        menu_File->setTitle(QApplication::translate("FEMSimulator", "&File", 0));
        menu_Test->setTitle(QApplication::translate("FEMSimulator", "&Test", 0));
        mainToolBar->setWindowTitle(QApplication::translate("FEMSimulator", "SimuPane", 0));
        restShapeToolBar->setWindowTitle(QApplication::translate("FEMSimulator", "Rest Shape Problem", 0));
        tetMeshStat->setWindowTitle(QApplication::translate("FEMSimulator", "Tetmesh Statistic", 0));
    } // retranslateUi

};

namespace Ui {
    class FEMSimulator: public Ui_FEMSimulator {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FEMSIMULATOR_H
