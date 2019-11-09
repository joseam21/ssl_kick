//
// Created by Ian Pérez on 10/21/19.
//

#include "RRTX.h"

RRTX::RRTX(float w, float h){
    eps = 5.0;
    delta = 10.0;
    r = 2 * delta;

    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::uniform_int_distribution<float> genW(0, w);
    std::uniform_int_distribution<float> genH(0, h);
    map_width = w;
    map_height = h;
    start = new Node(genW(engine), genH(engine)); // temporarily a random point
    robot = new Node(start->xcor, start->ycor);
    goal = new Node(75.0, float((int)h / 2));
    goal->g = 0;
    goal->lmc = 0;

    //Draw Yellow Circle of Start and End Goal

    V = std::vector<Node*>();
    V.push_back(goal);
    orphans = std::vector<Node*>();
    O = std::vector<Node*>();
    Q = MinHeap<CompTuple>();
}
///Generates a random point that does not collide with an obstacle
/// \returns A random valid Node
Node* RRTX::getRandomPoint() {
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::uniform_int_distribution<float> genW(0, map_width);
    std::uniform_int_distribution<float> genH(0, map_height);
    Node* counter = new Node(genW(engine), genH(engine));;
    while(!validNode(counter)){
        delete counter;
        counter = new Node(genW(engine), genH(engine));
    }
    return counter;
}

/// Checks if a point is in the map and not inside an obstacle
/// \param q A pointer to the Node that needs validation
/// \return True if valid, false otherwise
bool RRTX::validNode(Node *q) {
    if(0 <= q->xcor  && q->xcor < map_width && 0 <= q->ycor && q->ycor < map_height){
        for(Node* obstacle : O){
            if(q->distance(obstacle) <= 30) return false;
        }
        return true;
    }
    else return false;
}

void RRTX::step() {
    Node* v = getRandomPoint();
    int s = V.size();
    Node* v_nearest = nearest(v);
    if(v->distance(v_nearest) > delta){
        saturate(v, v_nearest);
    }
    if(validNode(v)){
        extend(v, r);
    }
    if(std::find(V.begin(), V.end(), v) != V.end()){
        rewireNeightbors(v);
        reduceInconsistency();
    }
}

/// Finds the nearest node to the input node v
/// \param v pointer to the input Node
/// \return The nearest node
Node* RRTX::nearest(Node* v) {
    Node* nearest = NULL;
    float best_dist = std::numeric_limits<float>::infinity();
    for(Node* node : V){
        float dist = v->distance(node);
        if(nearest == NULL || v->distance(node) < best_dist){
            nearest = node;
            best_dist = v->distance(node);
        }
    }
    return nearest;
}

/// Find's the best parent, as measured by lmc, for node v from the set of nodes U
/// \param v pointer to input Node
/// \param U Set of Nodes
void RRTX::findParent(Node* v, std::vector<Node*> U) {
    for(Node* u : U){
        if(possiblePath(v, u, O)){
            if(v->distance(u) <= r && v->lmc > v->distance(u) + u->lmc){
                v->parent = u;
                v->lmc = distance(v, u) + u->lmc;
            }
        }
    }
}

/// Determines if the straight-line path from node v to node u has an obstacle in the way
/// \param v Pointer to an input Node
/// \param u Pointer to an input Node
/// \param obstacles Set of obstacles
/// \return True if there is a path, False otherwise
bool RRTX::possiblePath(Node* v, Node* u, std::vector<Node*> obstacles) {
    float x1, y1, x2, y2;
    std::tie(x1, y1) = v->getCoords();
    std::tie(x2, y2) = u->getCoords();
    for(Node* obstacle : obstacles){
        if(v->distance(obstacle) <= 30 || u->distance(obstacle) <= 30){
            return false;
        }

        float x0, y0;
        std::tie(x0, y0) = obstacle->getCoords();
        std::tuple<float, float> a = std::make_tuple(x2 - x1, y2-y1);
        std::tuple<float, float> b = std::make_tuple(x0 - x1, y0-y1);

        float norm_a = std::sqrt(std::pow(std::get<0>(a), 2) + std::pow(std::get<1>(a), 2));
        if(norm_a == 0) return false;
        float comp = ((std::get<0>(a) * std::get<0>(b)) + (std::get<1>(a) * std::get<1>(b)))/norm_a;
        std::tuple<float, float> proj = std::make_tuple(comp * (std::get<0>(a)/norm_a), comp * (std::get<1>(a)/norm_a));

        float dx, dy;
        std::tie(dx, dy) = std::make_tuple(x1 + std::get<0>(proj), y1 + std::get<1>(proj));
        Node* qD = new Node(dx, dy);
        if(v->distance(qD) > v->distance(u)){
            continue;
        }
        if(obstacle->distance(qD) <= 30){
            return false;
        }
        delete qD;
    }
    return true;
}

/// Returns the Euclidean distance between v and u, but returns infinity if there is an obstacle in the way
/// \param v Pointer to an input Node
/// \param u Pointer to an input Node
/// \return The Euclidean distance between the two Nodes
float RRTX::distance(Node* v, Node* u) {
    if(possiblePath(v, u, O)){
        return v->distance(u);
    }
    return std::numeric_limits<float>::infinity();
}

/// Shifts the location of v so that it is self.delta away from v_nearest, but still in the same direction
/// \param v pointer to the input node
/// \param v_nearest pointer to the nearest node
void RRTX::saturate(Node* v, Node* v_nearest) {
    float heading = std::atan2(v->ycor - v_nearest->ycor, v->xcor - v_nearest->xcor);
    v->xcor = v_nearest->xcor + (delta * std::cos(heading));
    v->ycor = v_nearest->ycor + (delta * std::sin(heading));
}

/// Returns a set of all the nodes within radius r of node v
/// \param v Referene to the input Node
/// \param r Radius
/// \return Set of nodes near v
std::vector<Node*> RRTX::near(Node* v, float r) {
    std::vector<Node*> near_nodes = std::vector<Node*>();
    for(Node* node : V){
        if(v->distance(node) <= r){
            near_nodes.push_back(node);
        }
    }
    return near_nodes;
}

/// First determines if v can be added to the RRT. If it can be added, it inserts v in to the RRT
/// \param v pointer to the input Node
/// \param r Radius
void RRTX::extend(Node* v, float r) {
    std::vector<Node*> V_near = near(v, r);
    findParent(v, V_near);
    if(v->parent == NULL){
        return;
    }
    V.push_back(v);
    v->parent->children.push_back(v);

    for(Node* u : V_near){
        if(possiblePath(v, u, O)){
            v->outgoing_0.push_back(u);
            v->outgoing_r.push_back(u);
            u->incoming_r.push_back(v);
        }
        if(possiblePath(u, v, O)){
            u->outgoing_r.push_back(v);
            v->incoming_0.push_back(u);
            v->incoming_r.push_back(u);
        }
    }
}

void RRTX::cullNeighbors(Node* v, float r) { //Infinite Loop
    std::vector<Node*> s = std::vector<Node*>();
    for (auto p = v->outgoing_r.begin(); p != v->outgoing_r.end(); p++) {
        s.push_back(*p);
    }
    for (Node* u : s) {
        if(r < v->distance(u) && *(v->parent) != *u){
            v->outgoing_r.erase(std::remove_if(v->outgoing_r.begin(), v->outgoing_r.end(), [u](Node* n){ return *n == *u; }), v->outgoing_r.end());
            u->incoming_r.erase(std::remove_if(u->incoming_r.begin(), u->incoming_r.end(), [v](Node* n){ return *n == *v; }), u->incoming_r.end());
        }
    }
}

void RRTX::makeParentOf(Node* v, Node* u) {
    if(v->parent != NULL){
        v->parent->children.erase(std::remove_if(v->parent->children.begin(), v->parent->children.end(), [v](Node* n){ return *n == *v; }), v->parent->children.end());
    }
    v->parent = u;
    u->children.push_back(v);
    v->lmc = distance(v, u) + u->lmc;
}

void RRTX::rewireNeightbors(Node* v) {
    if(v->g - v->lmc > eps){
        cullNeighbors(v, r);
        std::vector<Node*> newSet = std::vector<Node*>();
        for(Node* uu : v->incoming_0){
            newSet.push_back(uu);
        }
        for(Node* uu : v->incoming_r){
            newSet.push_back(uu);
        }
        if(v->parent != NULL) {
            newSet.erase(std::remove_if(newSet.begin(), newSet.end(), [v](Node *n) { return *n == *(v->parent); }),
                         newSet.end());
        }
        for(Node* u : newSet){
            if(u->lmc > distance(u, v) + v->lmc){
                makeParentOf(u, v);
                if(u->g - u->lmc > eps){
                    verifyQueue(u);
                }
            }
        }
    }
}

bool RRTX::getBotCondition() {
    CompTuple top = Q.top();
    CompTuple robotKey = CompTuple{std::min(robot->g, robot->lmc), robot->g, robot};
    bool botCondition = (robot->g != robot->lmc || robot->g == std::numeric_limits<float>::infinity() || Q.find(robotKey) || top < robotKey);
    return botCondition;
}

void RRTX::reduceInconsistency() {
    while(Q.size() > 0 && getBotCondition()){
        CompTuple tup = Q.top();
        Q.pop();
        Node* v = tup.v;
        if(v->g - v->lmc > eps){
            updateLMC(v);
            rewireNeightbors(v);
        }
        v->g = v->lmc;
    }
}

void RRTX::updateLMC(Node* v) {
    cullNeighbors(v, r);
    Node* p = NULL;

    std::vector<Node*> newSet = std::vector<Node*>();
    for(Node* uu : v->outgoing_0){
        newSet.push_back(uu);
    }
    for(Node* uu : v->outgoing_r){
        newSet.push_back(uu);
    }
    for(auto it = orphans.begin(); it != orphans.end(); it++){
        auto res = std::find(newSet.begin(), newSet.end(), *it);
        if(res != newSet.end()){
            newSet.erase(res); //TODO: MIGHT NOT WORK
        }
    }
    for(Node* u : newSet){
        if(u->parent != v){
            if(v->lmc > distance(v, u) + u->lmc){
                p = u;
                break;
            }
        }
    }
    if(p != NULL){
        makeParentOf(v, p);
    }
}

void RRTX::updateObstacles(std::vector<Node*> removed, std::vector<Node*> added) {
    if(removed.size() > 0){
        for(Node* obs : removed){
            removeObstacle(obs);
        }
        reduceInconsistency();
    }
    if(added.size() > 0){
        for(Node* obs: added){
            addNewObstacle(obs);
        }
        propagateDescendants();
        reduceInconsistency();
    }
}

void RRTX::removeObstacle(Node* obs) {
    std::vector<std::tuple<Node*, Node*>> E_obs = std::vector<std::tuple<Node*, Node*>>();
    for(Node* v : V){
        for(Node* u : v->incoming_r){//near(v, r)){
            std::vector<Node*> newSet = {obs};
            if(!possiblePath(v, u, newSet)){
                E_obs.push_back(std::make_tuple(v, u));
            }
        }
    }
    O.erase(std::remove_if(O.begin(), O.end(), [obs](Node* n){return *obs == *n;}), O.end());
    std::vector<std::tuple<Node*, Node*>> E_obsCopy = std::vector<std::tuple<Node*, Node*>>();
    for (auto tup = E_obs.begin(); tup != E_obs.end(); tup++) {
        E_obsCopy.push_back(*tup);
    }
    for (std::tuple<Node*, Node*> tup : E_obsCopy) {
        Node* v = std::get<0>(tup);
        Node* u = std::get<1>(tup);
        if(!possiblePath(v, u, O)){
            E_obs.erase(std::remove_if(E_obs.begin(), E_obs.end(), [v, u](std::tuple<Node*, Node*> tup){return *(std::get<0>(tup)) == *v && *(std::get<1>(tup)) == *u;}), E_obs.end());
        }
    }
    for(std::tuple<Node*, Node*> tup : E_obs){
        Node* v = std::get<0>(tup);
        updateLMC(v);
        if(v->lmc != v->g || (v->lmc == v->g && v->g == std::numeric_limits<float>::infinity())){
            verifyQueue(v);
        }
    }
}

void RRTX::verifyQueue(Node* v) {
    CompTuple tup = CompTuple{ std::min(v->g, v->lmc), v->g, v};
    if(Q.find(tup)){
        bool found = false;
        std::vector<CompTuple> removed = std::vector<CompTuple>();
        while(!found){
            CompTuple u = Q.top();
            Q.pop();
            if(u.v == v){
                removed.push_back(tup);
                found = true;
            }
            else{
                removed.push_back(u);
            }
        }
        for(CompTuple u : removed){
            Q.push(u);
        }
    }
    else{
        Q.push(CompTuple{std::min(v->g, v->lmc), v->g, v});
    }
}

void RRTX::addNewObstacle(Node* obs) {
    O.push_back(obs);
    std::vector<std::tuple<Node*, Node*>> E_obs = std::vector<std::tuple<Node*, Node*>>();
    for(Node* v : V){
        std::vector<Node*> newSet = std::vector<Node*>();
        for(Node* uu : v->incoming_0){
            newSet.push_back(uu);
        }
        for(Node* uu : v->incoming_r){
            newSet.push_back(uu);
        }
        for(Node* u : newSet){
            if(!possiblePath(v, u, O)){
                E_obs.push_back(std::make_tuple(v, u));
            }
        }
    }
    for(std::tuple<Node*, Node*> tup : E_obs){
        Node* v = std::get<0>(tup);
        Node* u = std::get<1>(tup);
        if(v->parent != NULL && *(v->parent) == *u){
            u->children.erase(std::remove_if(u->children.begin(), u->children.end(), [v](Node* n){return *n == *v;}), u->children.end());
            v->parent = NULL;
            verifyOrphan(v);
        }
    }
}

void RRTX::propagateDescendants() {
    std::vector<Node*> stack;
    for(auto it = orphans.begin(); it != orphans.end(); it++){
        stack.push_back(*it);
    }
    while(stack.size() != 0){
        std::vector<Node*> new_stack = std::vector<Node*>();
        for(Node* orphan : stack){
            for(Node* u : orphan->children){
                orphans.push_back(u);
                new_stack.push_back(u);
            }
        }
        stack = new_stack;
    }
    for(Node* v : orphans){
        std::vector<Node*> newSet = std::vector<Node*>();
        for(Node* uu : v->outgoing_0){
            newSet.push_back(uu);
        }
        for(Node* uu : v->outgoing_r){
            newSet.push_back(uu);
        }
        if(v->parent != NULL){
            newSet.push_back(v->parent);
        }
        for(auto it = orphans.begin(); it != orphans.end(); it++){ //TODO: IMPROVE ALGORITHM, MORE EFFICIENT
            auto res = std::find(newSet.begin(), newSet.end(), *it);//newSet.find(*it);
            if(res != newSet.end()){
                newSet.erase(res);
            }
        }
        for(Node* u : newSet){
            u->g = std::numeric_limits<float>::infinity();
            verifyQueue(u);
        }
    }

    std::vector<Node*> orphanCopy;
    for(auto it = orphans.begin(); it != orphans.end(); it++){
        orphanCopy.push_back(*it);
    }
    for(Node* v : orphanCopy){
        orphans.erase(std::remove(orphans.begin(), orphans.end(), v), orphans.end());
        v->g = std::numeric_limits<float>::infinity();
        v->lmc = std::numeric_limits<float>::infinity();
        if(v->parent != NULL){
            v->parent->children.erase(std::remove_if(v->parent->children.begin(), v->parent->children.end(), [v](Node* n){return *n == *v;}), v->parent->children.end());
            v->parent = NULL;
        }
    }
}

void RRTX::verifyOrphan(Node* v) {
    CompTuple key = { std::min(v->g, v->lmc), v->g, v};
    if(Q.find(key)){
        Q.remove(key);
    }
    orphans.push_back(v);
}
