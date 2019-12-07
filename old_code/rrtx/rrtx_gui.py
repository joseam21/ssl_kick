#!/usr/bin/python
import tkinter
import time
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from Node import Node
#from queue import Queue, PriorityQueue
from MinHeap import MinHeap

# Here's the paper where we got this from: http://srl.informatik.uni-freiburg.de/teachingdir/ss15/otteWAFR14.pdf

class GUI:
    @staticmethod
    def drawRedCircle(x, y):
        circle = canvas.create_oval(0, 0, 0, 0, fill="red")
        canvas.coords(circle, x-2, y-2, x+2, y+2)

    @staticmethod
    def drawGreenCircle(x, y):
        circle = canvas.create_oval(0, 0, 0, 0, fill="green")
        canvas.coords(circle, x-30, y-30, x+30, y+30)

    @staticmethod
    def drawYellowCircle(x, y):
        circle = canvas.create_oval(0, 0, 0, 0, fill="yellow")
        canvas.coords(circle, x-2, y-2, x+2, y+2)

    @staticmethod
    def drawPinkCircle(x,y):
        circle = canvas.create_oval(0,0,0,0, fill='pink')
        canvas.coords(circle, x-30, y-30, x+30, y+30)

    @staticmethod
    def drawBlackCircle(x, y):
        circle = canvas.create_oval(0, 0, 0, 0, fill="black")
        canvas.coords(circle, x-2, y-2, x+2, y+2)

    @staticmethod
    def drawBlueCircle(x, y):
        circle = canvas.create_oval(0, 0, 0, 0, fill="blue")
        canvas.coords(circle, x-2, y-2, x+2, y+2)

    @staticmethod
    def drawBlackLine(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        line = canvas.create_line(x1, y1, x2, y2, fill="black")

    @staticmethod
    def drawRedLine(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        line = canvas.create_line(x1, y1, x2, y2, fill="red")

    @staticmethod
    def drawBlueLine(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        line = canvas.create_line(x1, y1, x2, y2, fill="blue")


class RRTX:
    def __init__(self, width, height):
        self.eps = 5.0
        self.delta = 10.0
        self.r = 2 * self.delta

        self.map_width = width
        self.map_height = height
        self.start = Node(random.randint(0, self.map_width),
                          random.randint(0, self.map_height))
        self.robot = self.start

        self.goal = Node(width // 2, height // 2)
        self.goal.g = 0
        self.goal.lmc = 0

        GUI.drawYellowCircle(*self.start.getIntCoords())
        GUI.drawYellowCircle(*self.goal.getIntCoords())

        self.V = {self.goal}
        self.orphans = set()
        self.O = set()
        self.Q = MinHeap()

    def drawObstacles(self):
        """
        Draws all the obstacles on the GUI
        """
        for obs in self.O:
            GUI.drawPinkCircle(*obs.getIntCoords())

    def getRandomPoint(self):
        """
        Generates a random point that does not collide with
        an obstacle
        """
        q = Node(random.randint(0, self.map_width),
                 random.randint(0, self.map_height))

        while not self.validNode(q):
            q = Node(random.randint(0, self.map_width),
                     random.randint(0, self.map_height))

        return q

    def validNode(self, q):
        """
        Checks if a point is in the map and not inside an obstacle
        """
        if 0 <= q.xcor < self.map_width and 0 <= q.ycor < self.map_height:
            for obstacle in self.O:
                if q.distance(obstacle) <= 30:
                    return False
            return True
        else:
            return False

    def step(self, obs_change=None, add_point=None):
        """
        Adds a point to the RRT, and updates the obstacles if there
        was a change
        """
        if obs_change:
            self.updateObstacles(obs_change[0], obs_change[1])

        if add_point == None:
            v = self.getRandomPoint()
        else:
            v = add_point
        v_nearest = self.nearest(v)

        if v.distance(v_nearest) > self.delta:
            self.saturate(v, v_nearest)

        if self.validNode(v):
            self.extend(v, self.r)

        if v in self.V:
            self.rewireNeighbors(v)
            self.reduceInconsistency()

    def nearest(self, v):
        """
        Finds the nearest node to the input node v
        """
        nearest = None
        best_dist = float('inf')

        for node in self.V:
            if nearest == None or v.distance(node) < best_dist:
                nearest = node
                best_dist = v.distance(node)

        return nearest

    def saturate(self, v, v_nearest):
        """
        Shifts the location of v so that it is self.delta away
        from v_nearest, but still in the same direction
        """
        heading = math.degrees(math.atan2(v.ycor - v_nearest.ycor,
                                          v.xcor - v_nearest.xcor))

        v.xcor = v_nearest.xcor + self.delta*math.cos(heading)
        v.ycor = v_nearest.ycor + self.delta*math.sin(heading)

    def near(self, v, r):
        """
        Returns a set of all the nodes within radius r of node v
        """
        near_nodes = set()
        for node in self.V:
            if v.distance(node) <= r:
                near_nodes.add(node)
        return near_nodes

    def extend(self, v, r):
        """
        First determines if v can be added to the RRT. If it can be
        added, it inserts v in to the RRT
        """
        V_near = self.near(v, r)
        self.findParent(v, V_near)
        if v.parent == None:
            return

        self.V.add(v)
        v.parent.children.add(v)

        for u in V_near:
            if self.possiblePath(v, u, self.O):
                v.outgoing_0.add(u)
                v.outgoing_r.add(u)
                u.incoming_r.add(v)
            if self.possiblePath(u, v, self.O):
                u.outgoing_r.add(v)
                v.incoming_0.add(u)
                v.incoming_r.add(u)

    def cullNeighbors(self, v, r):
        """
        Removes all neighbors of v that are now more than
        a radius r away
        """
        for u in v.outgoing_r.copy():
            if r < v.distance(u) and v.parent != u:
                v.outgoing_r.remove(u)
                u.incoming_r.remove(v)

    def makeParentOf(self, v, u):
        """
        Makes the node u the parent of node v
        """
        if v.parent != None:
            v.parent.children.remove(v)

        v.parent = u
        u.children.add(v)
        v.lmc = self.distance(v, u) + u.lmc

    def distance(self, v, u):
        """
        Returns the Euclidean distance between v and u, but
        returns infinity if there is an obstacle in the way
        """
        if self.possiblePath(v, u, self.O):
            return v.distance(u)
        return float('inf')

    def rewireNeighbors(self, v):
        """
        Rewires the RRT to use the new lmc value of node v
        """
        if v.g - v.lmc > self.eps:
            self.cullNeighbors(v, self.r)
            for u in (v.incoming_0 | v.incoming_r) - {v.parent}:
                if u.lmc > self.distance(u, v) + v.lmc:
                    self.makeParentOf(u, v)
                    if u.g - u.lmc > self.eps:
                        self.verifyQueue(u)

    def getBotCondition(self):
        """
        Returns the inconsistency conditions that depend on the robot's
        location
        """
        top = self.Q.getMin()
        robot_key = (min(self.robot.g, self.robot.lmc), self.robot.g, self.robot)
        bot_condition = (self.robot.g != self.robot.lmc or
                         self.robot.g == float('inf') or
                         robot_key in self.Q.queue or
                         top < robot_key)
        return bot_condition

    def reduceInconsistency(self):
        """
        While the graph is not self.epsilon-consistent: it fetches a node,
        it updates the node's lmc, rewires the neighbors, and makes that node
        0-consistent
        """
        while len(self.Q) > 0 and self.getBotCondition():
            _, _, v = self.Q.extractMin()
            if v.g - v.lmc > self.eps:
                self.updateLMC(v)
                self.rewireNeighbors(v)
            v.g = v.lmc

    def possiblePath(self, v, u, obstacles):
        """
        Determines if the straight-line path from node v to node u
        has an obstacle in the way
        """
        x1, y1 = v.getCoords()
        x2, y2 = u.getCoords()

        for obstacle in obstacles:
            if v.distance(obstacle) <= 30 or u.distance(obstacle) <= 30:
                return False

            try:
                x0, y0 = obstacle.getCoords()
                a = (x2-x1, y2-y1)
                b = (x0-x1, y0-y1)

                norm_a = (a[0]**2 + a[1]**2)**(1/2)
                comp = (a[0]*b[0] + a[1]*b[1])/norm_a
                proj = (comp*(a[0]/norm_a), comp*(a[1]/norm_a))

                dx, dy = x1+proj[0], y1+proj[1]
                qD = Node(dx, dy)
                if v.distance(qD) > v.distance(u):
                    continue

                if obstacle.distance(qD) <= 30:
                    return False
            except:
                return False

        return True

    def findParent(self, v, U):
        """
        Find's the best parent, as measured by lmc, for node v
        from the set of nodes U
        """
        for u in U:
            if self.possiblePath(v, u, self.O):
                if v.distance(u) <= self.r and v.lmc > v.distance(u) + u.lmc:
                    v.parent = u
                    v.lmc = self.distance(v, u) + u.lmc

    def updateObstacles(self, removed, added):
        """
        Updates the RRT with the knowledge that some obstacles have been
        removed, and some have been added
        """
        if len(removed) > 0:
            for obs in removed:
                self.removeObstacle(obs)
            self.reduceInconsistency()

        if len(added) > 0:
            for obs in added:
                self.addNewObstacle(obs)
            self.propagateDescendants()
            #self.verifyQueue(self.robot)
            self.reduceInconsistency()

    def propagateDescendants(self):
        """
        Starts with all initially orphaned/cut-off nodes and propagates
        that through their descendants so they're all marked as cut-off
        """
        stack = {orphan for orphan in self.orphans}
        while len(stack) != 0:
            new_stack = set()
            for orphan in stack:
                self.orphans.update(orphan.children)
                new_stack.update(orphan.children)
            stack = new_stack

        for v in self.orphans:
            for u in (v.outgoing_0 | v.outgoing_r | ({v.parent} if v.parent else set())) - self.orphans:
                u.g = float('inf')
                self.verifyQueue(u)

        for v in self.orphans.copy():
            self.orphans.remove(v)
            v.g = float('inf')
            v.lmc = float('inf')
            if v.parent != None:
                v.parent.children.remove(v)
                v.parent = None

    def verifyOrphan(self, v):
        """
        v is an orphan, so we take it out of the queue and put
        it in the orphans set
        """
        key = (min(v.g, v.lmc), v.g, v)
        if key in self.Q.heap:
            i = self.Q.heap.index(key)
            self.Q.deleteKey(i)
        self.orphans.add(v)

    def removeObstacle(self, obs):
        """
        Removes obstacle obs, and updates the RRT
        """
        E_obs = set()
        for v in self.V:
            for u in self.near(v, self.r):
                if not self.possiblePath(v, u, {obs}):
                    E_obs.add((v, u))

        self.O.discard(obs)

        for v, u in E_obs.copy():
            if not self.possiblePath(v, u, self.O):
                E_obs.remove((v, u))

        V_obs = {v for v, u in E_obs}
        for v in V_obs:
            self.updateLMC(v)
            if v.lmc != v.g or (v.lmc == v.g == float('inf')):
                self.verifyQueue(v)

    def addNewObstacle(self, obs):
        """
        Adds obstacle obs, and updates the RRT
        """
        self.O.add(obs)
        E_obs = set()
        for v in self.V:
            for u in (v.incoming_0 | v.incoming_r):
                if not self.possiblePath(v, u, self.O):
                    E_obs.add((v, u))

        for v, u in E_obs:
            if v.parent == u:
                u.children.remove(v)
                v.parent = None
                self.verifyOrphan(v)

    def verifyQueue(self, v):
        """
        If v is in the queue, update it's key. If it isn't,
        insert it into the queue
        """
        if (min(v.g, v.lmc), v.g, v) in self.Q.heap:
            found = False
            removed = []
            while not found:
                u = self.Q.extractMin()
                if u[2] == v:
                    removed.append((min(v.g, v.lmc), v.g, v))
                    found = True
                else:
                    removed.append(u)
            for u in removed:
                self.Q.insertKey(u)
        else:
            self.Q.insertKey((min(v.g, v.lmc), v.g, v))

    def updateLMC(self, v):
        """
        Updates the lmc of v based off of it's neighbors
        """
        self.cullNeighbors(v, self.r)
        p = None
        for u in (v.outgoing_0 | v.outgoing_r) - self.orphans:
            if u.parent != v:
                if v.lmc > self.distance(v, u) + u.lmc:
                    p = u
                    break
        if p:
            self.makeParentOf(v, p)

    def add_goal_point(self):
        """
        Draws the goal point
        """
        GUI.drawYellowCircle(*self.goal.getIntCoords())

    def drawTree(self):
        """
        Draws the RRT
        """
        for node in self.V:
            GUI.drawBlackCircle(*node.getIntCoords())
            if node.parent != None:
                GUI.drawBlackLine(node.getIntCoords(), node.parent.getIntCoords())
            if node.g == float('inf') and node.lmc == float('inf'):
                GUI.drawRedCircle(*node.getIntCoords())
        self.add_goal_point()

    def drawPathToGoal(self):
        """
        If self.robot is in the tree, it draws the path from robot
        to the goal
        """
        node = self.robot
        GUI.drawBlueCircle(*node.getIntCoords())
        while node.parent != None:
            GUI.drawBlueCircle(*node.getIntCoords())
            GUI.drawBlueLine(node.getIntCoords(), node.parent.getIntCoords())
            node = node.parent



height = 500
width = 800

top = tkinter.Tk()
canvas = tkinter.Canvas(top, bg="green", height=height, width=width)
canvas.pack()

r = RRTX(width, height)
def run_RRT():
    canvas.delete("all")

    #r = RRTX(width, height)
    cont = True
    i = 0
    while i < 3000:
        i += 1
        r.step()
        print(i)

    r.drawObstacles()
    r.drawTree()

def add_obstacles():
    canvas.delete("all")
    new_obs = set()
    for i in range(10):
        point = r.getRandomPoint()
        if point.distance(r.start) > 30 and point.distance(r.goal) > 30:
            new_obs.add(point)

    r.updateObstacles(set(), new_obs)
    print('updated')
    r.drawObstacles()
    print('obs drawn')
    r.drawTree()
    print('tree drawn')

def change_obstacles():
    canvas.delete("all")

    removed = {next(iter(r.O))}
    new_obs = set()
    while len(new_obs) == 0:
        point = r.getRandomPoint()
        if point.distance(r.start) > 30 and point.distance(r.goal) > 30:
            new_obs.add(point)

    r.updateObstacles(removed, new_obs)
    print('updated')
    r.drawObstacles()
    print('obs drawn')
    r.drawTree()
    print('tree drawn')

button = tkinter.Button(top, text="Restart", command=run_RRT)
button.pack()
button2 = tkinter.Button(top, text="Add Obs", command=add_obstacles)
button2.pack()
button3 = tkinter.Button(top, text="Move Obs", command=change_obstacles)
button3.pack()
top.mainloop()

