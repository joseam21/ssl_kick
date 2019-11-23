//
// Created by Ian Pérez on 11/18/19.
//

#ifndef RRTXOPTIMIZEDVISUALIZATION_VISUALIZER_H
#define RRTXOPTIMIZEDVISUALIZATION_VISUALIZER_H


#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QMainWindow>
#include <QPainter>
#include <QPushButton>
#include "RRTX.h"
#include <chrono>

QT_BEGIN_NAMESPACE
namespace Ui { class Visualizer; }
QT_END_NAMESPACE

class Visualizer : public QMainWindow
{
Q_OBJECT

public:
    RRTX* rrtx;
    Visualizer(QWidget *parent = nullptr);
    virtual ~Visualizer(){};
    void drawTree(QPainter* p);
    void drawObstacles(QPainter* p);
    void drawRobot(QPainter* p);
    void drawGoal(QPainter* p);
    void drawPathToGoal(QPainter* p);
    virtual void paintEvent(QPaintEvent *event);
private slots:
    void handleStart();
    void handleAddObstacle();
    void handleMoveObs();
    void handleMoveRobot();
private:
    Ui::Visualizer *ui;
    QPushButton* m_start;
    QPushButton* m_obstacles;
    QPushButton* m_moveObs;
    QPushButton* m_moveRobot;
};


#endif //RRTXOPTIMIZEDVISUALIZATION_VISUALIZER_H
