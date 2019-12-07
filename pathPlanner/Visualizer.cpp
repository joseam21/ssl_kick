//
// Created by Ian Pérez on 11/18/19.
//

#include "Visualizer.h"

Visualizer::Visualizer(QWidget *parent)
        : QMainWindow(parent)
{
    Node* newGoal = new Node(75.0, 300);
    rrtx = new RRTX(800, 500, NULL, newGoal);
//    rrtx->robot = rrtx->nearest(rrtxx->robot);
    m_start = new QPushButton("Start Tree", this);
    m_start->setGeometry(350, 520, 100, 50);
    connect(m_start, SIGNAL (clicked()), this, SLOT(handleStart()));

    m_obstacles = new QPushButton("Add Obstacle", this);
    m_obstacles->setGeometry(350, 570, 120, 50);
    connect(m_obstacles, SIGNAL (clicked()), this, SLOT(handleAddObstacle()));

    m_moveObs = new QPushButton("Move an Obstacle", this);
    m_moveObs->setGeometry(350, 630, 120, 50);
    connect(m_moveObs, SIGNAL (clicked()), this, SLOT(handleMoveObs()));

    m_moveRobot = new QPushButton("Move Robot", this);
    m_moveRobot->setGeometry(200, 630, 120, 50);
    connect(m_moveRobot, SIGNAL (clicked()), this, SLOT(handleMoveRobot()));
}

void Visualizer::handleStart() {
    auto t1 = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 9000; i++){
        rrtx->step();
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "TOTAL DURATION: " << duration << std::endl;
    Node* nearestToRobot = rrtx->V->nearest(rrtx->start);
    rrtx->robot = new Node(nearestToRobot->xcor, nearestToRobot->ycor);
    rrtx->robot->parent = nearestToRobot->parent;
    std::cout << "END" << std::endl;
    this->repaint();
}

void Visualizer::handleAddObstacle() {
    Node* point = rrtx->getRandomPoint();
    while(!rrtx->validNode(point) && point->distance(rrtx->start) < 30 && point->distance(rrtx->goal) < 30){
        delete point;
        point = rrtx->getRandomPoint();
    }
    if(point->distance(rrtx->start) > 30 && point->distance(rrtx->goal) > 30){
        rrtx->updateObstacles(std::vector<Node*>(), std::vector<Node*>{point});
    }
    this->repaint();
}

void Visualizer::handleMoveObs(){
    if(rrtx->O.size() <= 0) return;
    std::vector<Node*> add = std::vector<Node*>();
    std::vector<Node*> remove = std::vector<Node*>();
    for(int i = 0; i < rrtx->O.size(); i++) {
        Node *obstaclePointer = rrtx->O[i];
        Node *newObs = rrtx->getRandomPoint();
        add.push_back(newObs);
        remove.push_back(obstaclePointer);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    rrtx->updateObstacles(remove, add);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "TOTAL DURATION: " << duration << std::endl;
    this->repaint();
}

void Visualizer::handleMoveRobot() {
    if(*rrtx->robot == *rrtx->goal) return;
    Node* newN = new Node(rrtx->robot->parent->xcor, rrtx->robot->parent->ycor);
    newN->parent = rrtx->robot->parent->parent;
    rrtx->robot = newN;

    std::vector<Node*> nextNodes = std::vector<Node*>();
    nextNodes = rrtx->getNextPoints();

    std::cout << "Next 5 Points: " << std::endl;

    for(Node* n : nextNodes){
        n->display();
    }

    this->repaint();
}

void Visualizer::drawTree(QPainter* qPainter) {
    QPen blackPen(Qt::black);
    QPen redPen(Qt::red);
    QBrush blackBrush(Qt::black);
    QBrush redBrush(Qt::red);
    std::vector<Node*> nodes;
    rrtx->V->list(nodes);
    for(auto node : nodes){
        int x, y;
        std::tie(x, y) = node->getIntCoords();
        if(node->parent != NULL){
            int x1, y1;
            std::tie(x1, y1) = node->parent->getIntCoords();
            qPainter->setPen(blackPen);
            qPainter->drawLine(x, y, x1, y1);
        }
        if(node->g == std::numeric_limits<float>::infinity() && node->lmc == std::numeric_limits<float>::infinity()){
            qPainter->setBrush(redBrush);
            qPainter->setPen(redPen);
            qPainter->drawEllipse(x, y, 3, 3);
        }
        else{
            int x, y;
            std::tie(x, y) = node->getIntCoords();
            qPainter->setBrush(blackBrush);
            qPainter->setPen(blackPen);
            qPainter->drawEllipse(x, y, 3, 3);
        }
    }
}

void Visualizer::drawObstacles(QPainter* qPainter) {
    for(Node* node : rrtx->O){
        qPainter->setBrush(Qt::green);
        int x, y;
        std::tie(x, y) = node->getIntCoords();
        qPainter->drawEllipse(QPoint(x, y), 30, 30);
    }
}

void Visualizer::drawRobot(QPainter* qPainter){
    qPainter->setBrush(Qt::yellow);
    int x, y;
    std::tie(x, y) = rrtx->robot->getIntCoords();
    qPainter->drawEllipse(QPoint(x, y), 20, 20);
}

void Visualizer::drawGoal(QPainter* qPainter){
    qPainter->setBrush(Qt::blue);
    int x, y;
    std::tie(x, y) = rrtx->goal->getIntCoords();
    qPainter->drawEllipse(QPoint(x, y), 20, 20);
}
void Visualizer::drawPathToGoal(QPainter* qPainter){
    if(rrtx->V->size() < 1000) return;
    QPen p = QPen(Qt::magenta);
    p.setWidth(5);
    qPainter->setPen(p);
    Node* current = rrtx->robot;
    while(*current != *rrtx->goal){
        Node* parent = current->parent;
        if(parent == NULL) return;
        qPainter->drawLine(current->xcor, current->ycor, parent->xcor, parent->ycor);
        current = parent;
    }
}

void Visualizer::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    drawObstacles(&painter);
    drawPathToGoal(&painter);
    drawTree(&painter);
    drawGoal(&painter);
    drawRobot(&painter);
}
