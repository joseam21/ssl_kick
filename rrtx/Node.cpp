//
// Created by Ian Pérez on 10/26/19.
//

#include "Node.h"

Node::Node(float xc, float yc){
    xcor = xc;
    ycor = yc;

    incoming_0 = std::vector<Node*>();
    outgoing_0 = std::vector<Node*>();

    incoming_r = std::vector<Node*>();
    outgoing_r = std::vector<Node*>();

    g = std::numeric_limits<float>::infinity();
    lmc = std::numeric_limits<float>::infinity();
    parent = NULL;
    children = std::vector<Node*>();
}

Node::Node(){
    xcor = -100;
    ycor = -100;
}

std::tuple<float, float> Node::getCoords(){
    return std::make_tuple(xcor, ycor);
}

std::tuple<int, int> Node::getIntCoords(){
    return std::make_tuple((int)xcor, (int)ycor);
}

float Node::distance(Node* other){
    float other_x, other_y;
    std::tie(other_x, other_y) = other->getCoords();
    return std::sqrt(std::pow((xcor - other_x), 2) + std::pow((ycor - other_y), 2));
}

bool Node::operator==(const Node &other) const {
    return this->xcor == other.xcor && this->ycor == other.ycor;
}

bool Node::operator!=(const Node &other) const {
    return this->xcor != other.xcor || this->ycor != other.ycor;
}

bool Node::operator<(const Node &other) const {
    if(this->xcor == other.xcor){
        return this->ycor < other.ycor;
    }
    return this->xcor < other.xcor;
}

bool Node::operator>(const Node &other) const {
    return true;
}

void Node::display() {
    std::cout << std::string("\n") + "(" + std::to_string(xcor) + ", " + std::to_string(ycor) + ")\n";
}