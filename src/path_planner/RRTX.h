//
// Created by Ian Pérez on 11/18/19.
//

#ifndef RRTXOPTIMIZEDVISUALIZATION_RRTX_H
#define RRTXOPTIMIZEDVISUALIZATION_RRTX_H


#include "Node.h"
#include <chrono>
#include <queue>
#include <vector>
#include <random>
#include <tuple>
#include <algorithm>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <boost/heap/fibonacci_heap.hpp>

using namespace boost;

class RRTX {
    typedef boost::heap::fibonacci_heap<Node *,
            boost::heap::compare<node_compare> > Queue;
    typedef Queue::handle_type handle_t;
    typedef boost::unordered_map<Node *, handle_t> NodeMap;
    typedef boost::unordered_map<Node *, bool> OrphanMap;
public:
    RRTX(float width, float height, Node* starting, Node* goal);
    float eps;
    float delta;
    float r;
    float gamd;
    float lam;

    float map_width;
    float map_height;
    Node* start;
    Node* robot;
    Node* goal;

    std::vector<Node*> O;
    ompl::NearestNeighborsGNAT<Node *> *V;
    OrphanMap orphanHash;
    Queue Q;
    NodeMap nodeHash;
    Node* getRandomPoint();
    bool validNode(Node* q);
    void step();
    //Left step function out
    Node* nearest(Node* v);
    void saturate(Node* v, Node* v_nearest);
    std::vector<Node*> near(Node* v, float r);
    void extend(Node* v, float r);
    void removeNeighbors(std::vector<Node *> &neighbors, Node *toRemove);
    void cullNeighbors(Node* v, float r);
    void makeParentOf(Node* v, Node* u);
    float distance(Node* v, Node* u);
    void rewireNeightbors(Node* v);
    bool getBotCondition();
    void reduceInconsistency();
    bool possiblePath(Node* v, Node* u, std::vector<Node*> obstacles);
    void findParent(Node* v, std::vector<Node*> U);
    void updateObstacles(std::vector<Node*> removed, std::vector<Node*> added);
    void propagateDescendants();
    void verifyOrphan(Node* v);
    void removeObstacle(Node* obs);
    void addNewObstacle(Node* obs);
    void verifyQueue(Node* v);
    void updateLMC(Node* v);
    void updateRadius();
    void queueInsert(Node *v);
    bool queueContains(Node *v);
    void queueUpdate(Node *v);
    void queueRemove(Node *v);
    Node* queuePop();
    void updateKey(Node *v);
    void insertOrphanChildren(Node *v);
    std::vector<Node*> getNextPoints(int num = 5);
};


#endif //RRTXOPTIMIZEDVISUALIZATION_RRTX_H
