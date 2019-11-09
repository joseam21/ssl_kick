//
// Created by Ian Pérez on 10/21/19.
//

#ifndef TESTINGRRTX_RRTX_H
#define TESTINGRRTX_RRTX_H


#include "Node.h"
#include <queue>
#include <vector>
#include <random>
#include <tuple>
#include <algorithm>

struct CompTuple{
    float min;
    float g;
    Node* v;

    CompTuple(float min, float g, Node* ve)
    {
        v = ve;
        this->g = g;
        this->min = min;
    }

    bool operator<(const CompTuple& tup){
        if(this->min == tup.min){
            return this->g < tup.g;
        }
        return this->min < tup.min;
    }
    bool operator>(const CompTuple& tup){
        if(this->min == tup.min){
            return this->g > tup.g;
        }
        return this->min > tup.min;
    }
    const bool operator==(const CompTuple& tup){
        return this->min == tup.min && this->g == tup.g && this->v == tup.v;
    }
};

struct CompareStructs{
    bool operator() (CompTuple const& c1, CompTuple const& c2){
        if(c1.min != c2.min){
            return c1.min > c2.min;
        }
        else {
            return c1.g > c2.g;
        }
    }
};

template<typename T>
class MinHeap : public std::priority_queue<T, std::vector<T>, CompareStructs> //TODO: CHECK IF CUSTOM PQ WORKS
{
public:

    bool remove(const T& value) {
        auto it = std::find(this->c.begin(), this->c.end(), value);
        if (it != this->c.end()) {
            this->c.erase(it);
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
            return true;
        }
        else {
            return false;
        }
    }
    bool find(const T& value) {
        auto it = std::find(this->c.begin(), this->c.end(), value);
        if (it != this->c.end()) {
            return true;
        }
        else {
            return false;
        }
    }
    bool removeByNode(const Node& value) {
        auto it = std::find_if(this->c.begin(), this->c.end(), [value](CompTuple s){return *s.v == value;});
        if (it != this->c.end()) {
            this->c.erase(it);
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
            return true;
        }
        else {
            return false;
        }
    }
};

class RRTX {
public:
    RRTX(float width, float height);
    float eps;
    float delta;
    float r;

    float map_width;
    float map_height;
    Node* start;
    Node* robot;
    Node* goal;

    std::vector<Node*> O;
    std::vector<Node*> V;
    std::vector<Node*> orphans;
    MinHeap<CompTuple> Q;
//    std::priority_queue<CompTuple, std::vector<CompTuple>, CompareStucts> Q;
//    std::vector<CompTuple> Q_vect;
    //void drawObstacles(); TODO: Implement drawing
    Node* getRandomPoint();
    bool validNode(Node* q);
    void step();
    //Left step function out
    Node* nearest(Node* v);
    void saturate(Node* v, Node* v_nearest);
    std::vector<Node*> near(Node* v, float r);
    void extend(Node* v, float r);

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

    // add_goal_point
    //drawTree
    //drawPathToGoal
};

#endif //TESTINGRRTX_RRTX_H
