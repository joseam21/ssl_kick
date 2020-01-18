//
// Created by Ian Pérez on 11/18/19.
//

//
// Created by Ian Pérez on 10/21/19.
//

#include "RRTX.h"

RRTX::RRTX(float xmin, float xmax, float ymin, float ymax, Node* start, Node* goal){
    // Initializing hyper-parameters
    eps = 0.073; //TODO: Author suggests this to be width/2 or (safe distace)/2
    delta = 2.75;
    r = delta;
    lam = 1300;
    gamd = 100;

    // Initializing random point generator for map
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::uniform_real_distribution<float> genW(xmin, xmax);
    std::uniform_real_distribution<float> genH(ymin, ymax);
    initial = start;
    robot = start;
    map_width = xmax - xmin;
    map_height = ymax - ymin;
    x_min = xmin;
    x_max = xmax;
    y_min = ymin;
    y_max = ymax;

    // Initializing goal node as root of tree
    this->goal = goal;
    this->goal->g = 0;
    this->goal->lmc = 0;

    // Initializing necessary data structures
    O = std::vector<Node*>();
    V = new ompl::NearestNeighborsGNAT<Node*>();
    V->setDistanceFunction([this](const Node *v, const Node *u){
        return std::sqrt(((v->xcor - u->xcor) * (v->xcor - u->xcor)) + ((v->ycor - u->ycor) * (v->ycor - u->ycor)));
    });
    V->add(goal);
    orphanHash = OrphanMap();
    Q = Queue();
    nodeHash = NodeMap();

    //// Creating tree to fill space
    //for(int i = 0; i < 2000; ++i) {
    //    step();
    //}

    initial = start;
    Node* nearestToRobot = V->nearest(start);
    robot = new Node(nearestToRobot->xcor, nearestToRobot->ycor);
    robot->parent = nearestToRobot->parent;
}

///Generates a random point that does not collide with an obstacle
/// \returns A random valid Node
Node* RRTX::getRandomPoint() {
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::uniform_real_distribution<float> genW(x_min, x_max);
    std::uniform_real_distribution<float> genH(y_min, y_max);
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
            if(q->distance(obstacle) <= 0.146) return false;
        }
        return true;
    }
    else return false;
}

void RRTX::updateRadius(){ //TODO: Verify Implementation
    r = std::min(pow((lam/gamd) * log(std::max(1.0*V->size(), 2.0))/(1.0*V->size()), 1.0/2.0), 1.0*delta);
}


void RRTX::step() {
    updateRadius();
    Node* v = getRandomPoint();
    int s = V->size();
    Node* v_nearest = V->nearest(v);
    if(v->distance(v_nearest) > delta){
        saturate(v, v_nearest);
    }
    if(validNode(v)){
        extend(v, r);
        if(v->parent != NULL){
            rewireNeightbors(v);
            reduceInconsistency();
        }
    }
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
        if(v->distance(obstacle) <= 0.146 || u->distance(obstacle) <= 0.146){
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
        if(obstacle->distance(qD) <= 0.146){
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

/// First determines if v can be added to the RRT. If it can be added, it inserts v in to the RRT
/// \param v pointer to the input Node
/// \param r Radius
void RRTX::extend(Node* v, float r) {
    std::vector<Node *> V_near;
    V->nearestR(v, r, V_near);
    findParent(v, V_near);
    if(v->parent == NULL){
        return;
    }
    V->add(v);
    v->parent->children.push_back(v);

    for(Node* u : V_near){
        if(possiblePath(v, u, O)){
            v->outgoing_0.push_back(u);
            u->incoming_r.push_back(v);
        }
        if(possiblePath(u, v, O)){
            u->outgoing_r.push_back(v);
            v->incoming_0.push_back(u);
        }
    }
}

void RRTX::removeNeighbors(std::vector<Node *> &neighbors, Node *toRemove) {
    for(auto it = neighbors.begin(); it != neighbors.end(); it++){
        if(*(*it) == *toRemove){
            neighbors.erase(it);
            return;
        }
    }
}

void RRTX::cullNeighbors(Node* v, float r) { //Infinite Loop
    auto it = v->outgoing_r.begin();
    while(it != v->outgoing_r.end()){
        Node *u = *it;
        if(v->parent != u && r < distance(v, u)){
            it = v->outgoing_r.erase(it);
            removeNeighbors(u->incoming_r, v);
        }
        else{
            it++;
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
        std::vector<Node*> inSet = v->inNeighbors();
        if(v->parent != NULL) {
            inSet.erase(std::remove_if(inSet.begin(), inSet.end(), [v](Node *n) { return *n == *(v->parent); }),
                         inSet.end());
        }
        for(Node* u : inSet){
            if(u->lmc > distance(u, v) + v->lmc){
                u->lmc = distance(u, v) + v->lmc; //TODO: Not in original Ask Why
                makeParentOf(u, v);
                if(u->g - u->lmc > eps){
                    verifyQueue(u);
                }
            }
        }
    }
}

bool RRTX::getBotCondition() {
    NodeKey top = Q.top()->key;
    NodeKey robotKey = NodeKey(std::min(robot->g, robot->lmc), robot->g);
    bool botCondition = (robot->g != robot->lmc || robot->g == std::numeric_limits<float>::infinity() || queueContains(robot) || top < robotKey);
    return botCondition;
}
//TODO: Change Queue to Boost:FibonacciHeap
//TODO: Change V to be a NearestNeighborTree
void RRTX::reduceInconsistency() {
    while(Q.size() > 0 && getBotCondition()){
        Node *v = queuePop();
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

    std::vector<Node*> outSet = v->outNeighbors();
    for(Node *u : outSet){
        if(orphanHash.find(u) != orphanHash.end() || u->parent == v)
            continue;
        if(v->lmc > distance(v, u) + u->lmc){
            p = u;
        }
    }
    if(p != NULL)
        makeParentOf(v, p);
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
    std::vector<Node*> nodes;
    V->list(nodes);
    for(auto v : nodes){
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
    if(queueContains(v)) queueUpdate(v);
    else queueInsert(v);
}

void RRTX::queueInsert(Node *v) {
    updateKey(v);
    nodeHash[v] = Q.push(v);
}
bool RRTX::queueContains(Node *v) {
    return nodeHash.find(v) != nodeHash.end();
}
void RRTX::queueUpdate(Node *v) {
    updateKey(v);
    Q.update(nodeHash[v]);
}
void RRTX::queueRemove(Node *v) {
    Q.erase(nodeHash[v]);
    nodeHash.erase(v);
}
Node* RRTX::queuePop() {
    Node *toRmv = Q.top();
    nodeHash.erase(toRmv);
    Q.pop();
    return toRmv;
}
void RRTX::updateKey(Node *v) {
    v->key.k1 = std::min(v->g, v->lmc);
    v->key.k2 = v->g;
}

void RRTX::addNewObstacle(Node* obs) {
    O.push_back(obs);
    std::vector<std::tuple<Node*, Node*>> E_obs = std::vector<std::tuple<Node*, Node*>>();
    std::vector<Node*> nodes;
    V->list(nodes);
    for(auto v : nodes){
        std::vector<Node*> inSet = v->inNeighbors();
        for(Node* u : inSet){
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
    std::vector<Node *> orphans;
    orphans.reserve(orphanHash.size());
    for(auto o: orphanHash){
        orphans.push_back(o.first);
    }
    for(auto v : orphans){
        insertOrphanChildren(v);
    }
    orphans.clear();
    orphans.reserve(orphanHash.size());
    for(auto v : orphanHash){
        orphans.push_back(v.first);
    }
    for(auto v : orphans){
        std::vector<Node *> neighbors = v->outNeighbors();
        if(v->parent != NULL) neighbors.push_back(v->parent);
        for(auto n : neighbors){
            if(orphanHash.find(n) != orphanHash.end()) continue;
            n->g = std::numeric_limits<float>::infinity();
            verifyQueue(n);
        }
    }
    for(auto v : orphans){
        v->g = std::numeric_limits<float>::infinity();
        v->lmc = std::numeric_limits<float>::infinity();
        if(v->parent != NULL){
            auto &children = v->parent->children;
            children.erase(std::remove_if(children.begin(), children.end(), [v](Node *c) {return v == c;}), children.end());
            v->parent = NULL;
        }
    }
    orphanHash.clear();
}

void RRTX::verifyOrphan(Node* v) {
    if(queueContains(v)){
        queueRemove(v);
    }
    orphanHash[v] = true;
}

void RRTX::insertOrphanChildren(Node *v) {
    orphanHash[v] = true;
    for(auto c : v->children){
        insertOrphanChildren(c);
    }
}


/// Returns the next num nodes from the path of the robot
/// \param num Number of points to return
/// \return std::vector<Node*> with the nodes
std::vector<Node*> RRTX::getNextPoints(int num) {
    std::vector<Node*> result = std::vector<Node*>();
    result.reserve(num);

    Node* newN = robot->parent;
    if (newN == NULL) {
        return result;
    }

    int i = 0;
    while (newN != NULL && i < num){
        result.push_back(newN);
        ++i;
        newN = newN->parent;
    }

    return result;
}



