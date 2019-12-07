//
// Created by Ian Pérez on 11/18/19.
//

#ifndef RRTXOPTIMIZEDVISUALIZATION_NODE_H
#define RRTXOPTIMIZEDVISUALIZATION_NODE_H


#include <vector>
#include <tuple>
#include <limits>
#include <cmath>
#include <string>
#include <iostream>
#include <boost/unordered_map.hpp>

struct NodeKey {
    float k1 = std::numeric_limits<float>::infinity();
    float k2 = std::numeric_limits<float>::infinity();

    NodeKey(float first, float second){
        k1 = first;
        k2 = second;
    }
    NodeKey(){
        k1 = std::numeric_limits<float>::infinity();
        k2 = std::numeric_limits<float>::infinity();
    }

    bool operator<(const NodeKey key) const
    {
        return k1 < key.k1 || abs(k1 - key.k1) < 0.00001 && k2  < key.k2;
    }
    bool operator>(const NodeKey key) const
    {
        return k1 > key.k1 || abs(k1 - key.k1) < 0.00001 && k2  > key.k2;
    }
};

class Node {
public:
    float xcor;
    float ycor;

    NodeKey key;

    std::vector<Node*> incoming_0;
    std::vector<Node*> outgoing_0;

    std::vector<Node*> incoming_r;
    std::vector<Node*> outgoing_r;

    float g;
    float lmc;

    Node* parent;
    std::vector<Node*> children;

    Node(float xc, float yc);
    Node();
    std::tuple<float, float> getCoords();
    std::tuple<int, int> getIntCoords();
    float distance(Node* other);
    const float constDistance(const Node* other);
    std::vector<Node*> inNeighbors();
    std::vector<Node*> outNeighbors();
    void display(); //For debugging purposes

    bool operator==(const Node &other) const;
    bool operator<(const Node &other) const;
    bool operator>(const Node &other) const;
    bool operator!=(const Node &other) const;

};

typedef std::pair<Node *, Node *> NodePair;
struct hash_node_pair
{
    std::size_t operator()(const NodePair &p) const
    {
        std::size_t seed = 0;
        seed += boost::hash<Node *>()(p.first);
        seed ^= boost::hash<Node *>()(p.second);
        return seed;
    }
};

struct node_pair_equal
{
    bool operator()(const NodePair &p1, const NodePair &p2) const
    {
        return  p1.first == p2.first && p1.second == p2.second;
    }
};

struct node_compare
{
    bool operator()(const Node *v1, const Node *v2) const
    {
        return v1->key > v2->key;
    }
};


#endif //RRTXOPTIMIZEDVISUALIZATION_NODE_H
