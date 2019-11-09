//
// Created by Ian Pérez on 10/19/19.
//

#ifndef RRTXC_NODE_H
#define RRTXC_NODE_H

#include <vector>
#include <tuple>
#include <limits>
#include <cmath>
#include <string>
#include <iostream>

class Node {
public:
    float xcor;
    float ycor;

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
    void display(); //For debugging purposes

    bool operator==(const Node &other) const;
    bool operator<(const Node &other) const;
    bool operator>(const Node &other) const;
    bool operator!=(const Node &other) const;

};

#endif //RRTXC_NODE_H
