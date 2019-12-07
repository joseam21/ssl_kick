#!/usr/bin/python
import tkinter
import time
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from Node import Node

class GUI:
    @staticmethod
    def drawRedCircle(x, y):
        circle = canvas.create_oval(0, 0, 0, 0, fill="red")
        canvas.coords(circle, x-2, y-2, x+2, y+2)

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

class RRT:
    def __init__(self, width, height):
        self.start = Node(random.randint(0, width),
                          random.randint(0, height))
        self.goal = Node(random.randint(0, width),
                         random.randint(0, height))
        self.start.setH(self.start.distance(self.goal))
        
        self.tree_points = [self.start]
        self.obstacles = []
        self.delta = 10.0
        self.neighborhood = 2.0 * self.delta
        
        self.width = width
        self.height = height
        
        # draw start and end
        GUI.drawYellowCircle(self.start.xcor, self.start.ycor)
        GUI.drawYellowCircle(self.goal.xcor, self.goal.ycor)
        
        # generate and draw obstacles
        self.generateRandomObstacles(30)
        for obstacle in self.obstacles:
            GUI.drawPinkCircle(obstacle.xcor, obstacle.ycor)

    def obstacleFree(self, q1, q2):
        x1, y1 = q1.xcor, q1.ycor
        x2, y2 = q2.xcor, q2.ycor
        
        for obstacle in self.obstacles:
            try:
                x0, y0 = obstacle.xcor, obstacle.ycor
                a = (x2-x1, y2-y1)
                b = (x0-x1, y0-y1)

                norm_a = (a[0]**2 + a[1]**2)**(1/2)
                comp = (a[0]*b[0] + a[1]*b[1])/norm_a
                proj = (comp*(a[0]/norm_a), comp*(a[1]/norm_a))

                dx, dy = x1+proj[0], y1+proj[1]
                qD = Node(dx, dy)
                if q1.distance(qD) > q1.distance(q2):
                    continue

                if obstacle.distance(qD) <= 30:
                    return False
            except:
                return False

        return True


    def getRandomPoint(self):
        q = Node(random.randint(0, self.width),
                 random.randint(0, self.height))
        
        while not self.validNode(q):
            q = Node(random.randint(0, self.width),
                     random.randint(0, self.height))

        return q
   
    def getBiasedRandomPoint(self):
        ordered = sorted(self.tree_points, key=lambda node: node.getH())
        for node in ordered:
            if node.isEnd():
                continue
            
            heading = math.degrees(math.atan2(self.goal.ycor - node.ycor,
                                              self.goal.xcor - node.xcor))
            heading += random.uniform(-180.0, 180.0)
            q = Node(node.xcor+(self.delta*math.cos(heading)),
                     node.ycor+(self.delta*math.sin(heading)))
            q.setParent(node)

            if self.validNode(q):
                return q

        print("rip")
        return Node(-1, -1)

    def generateRandomObstacles(self, num):
        for i in range(num):
            point = self.getRandomPoint()
            if point.distance(self.start) > 30 and point.distance(self.goal) > 30:
                self.obstacles.append(point)

    def validNode(self, q):
        if 0 <= q.xcor < self.width and 0 <= q.ycor < self.height:
            for obstacle in self.obstacles:
                if q.distance(obstacle) <= 30:
                    return False
            return True
        else:
            return False

    def endOfPath(self, q, radius):
        for i1 in range(q.getIntCoord()[0] - radius, q.getIntCoord()[0] + radius):
            for j1 in range(q.getIntCoord()[1] - radius, q.getIntCoord()[1] + radius):
                if i1 in range(0, width) and j1 in range(0, height):
                    if not self.validNode(Node(i1, j1)):
                        q.setEnd(True)
                        return True
        return False

    def add_point(self):
        """
        Adds a point to the RRT tree, returns True if more points need to be 
        added to reach the goal, returns False otherwise.
        """
        # Generate a new point that is in bounds and not in an obstacle
        if random.randint(0, 10) <= 5:
            q_new = self.getRandomPoint()
        else:
            q_new = self.getBiasedRandomPoint()

        #if self.endOfPath(q_new, 8):
        #    self.start.setEnd(False)
        #    return True
        
        # Find closest node to q_new
        q_nearest = self.tree_points[-1]
        best_distance = q_new.distance(q_nearest)
        for node in self.tree_points[:-1]:
            if q_new.distance(node) <= best_distance:
                q_nearest = node
                best_distance = q_new.distance(node)

        # Slide q_new closer
        heading = math.degrees(math.atan2(q_new.ycor - q_nearest.ycor,
                                          q_new.xcor - q_nearest.xcor))
        
        q_new = Node(q_nearest.xcor+(self.delta*math.cos(heading)),
                     q_nearest.ycor+(self.delta*math.sin(heading)))
        q_new.setH(q_new.distance(self.goal))
        
        if not self.validNode(q_new):
            return True

        #if len(close_nodes) > 20:
        #    q_new.setEnd(True)
        #    for node in close_nodes:
        #        q_new.setEnd(True)
        #    self.start.setEnd(False)
        #    return True

        # Find all nodes within self.neighborhood of q_new
        close_nodes = []
        for node in self.tree_points:
            if q_new.distance(node) <= self.neighborhood:
                close_nodes.append(node)

        # Find cheapest parent for q_new from close_nodes
        best_parent = close_nodes[0]
        for node in close_nodes[1:]:
            cost1 = q_new.distance(best_parent) + best_parent.getCost()
            cost2 = q_new.distance(node) + node.getCost()
            if cost2 < cost1:
                best_parent = node

        if self.obstacleFree(best_parent, q_new):
            q_new.setParent(best_parent)
            self.tree_points.append(q_new)

            # Look at close_nodes and see if any of them have a better path through q_new
            for node in close_nodes:
                cost1 = node.getCost()
                cost2 = q_new.getCost() + q_new.distance(node)
                if cost2 < cost1:
                    if self.obstacleFree(q_new, node):
                        node.setParent(q_new)

            # Determine if more points need to be added
            if q_new.distance(self.goal) <= 10:
                return False
        
        return True


    def add_goal_point(self):
        close_nodes = []
        radius = self.neighborhood
        while len(close_nodes) == 0:
            for node in self.tree_points:
                if self.goal.distance(node) <= radius:
                    close_nodes.append(node)
            radius *= 2

        best_parent = close_nodes[0]
        for node in close_nodes[1:]:
            cost1 = self.goal.distance(best_parent) + best_parent.getCost()
            cost2 = self.goal.distance(node) + node.getCost()
            if cost2 < cost1:
                best_parent = node

        self.goal.setParent(best_parent)
        self.tree_points.append(self.goal)
        
    def drawTree(self):
        for node in self.tree_points:
            GUI.drawBlackCircle(node.xcor, node.ycor)
            for child in node.getChildren():
                GUI.drawBlackLine(node.getIntCoord(), child.getIntCoord())

        for node in self.tree_points:
            if node.isEnd():
                GUI.drawBlueCircle(node.xcor, node.ycor)

    def drawPathToGoal(self):
        node = self.goal
        while node.getParent() != False:
            GUI.drawRedCircle(node.xcor, node.ycor)
            GUI.drawRedLine(node.getIntCoord(), node.getParent().getIntCoord())
            node = node.getParent()

height = 500
width = 800

top = tkinter.Tk()
canvas = tkinter.Canvas(top, bg="green", height=height, width=width)
canvas.pack()

def run_RRT():
    canvas.delete("all")

    r = RRT(width, height)
    cont = True
    i = 0
    while cont and i < 3000:
        cont = r.add_point()
        i += 1
        print(i)
    r.add_goal_point()

    r.drawTree()
    r.drawPathToGoal()
    GUI.drawYellowCircle(r.start.xcor, r.start.ycor)

button = tkinter.Button(top, text="Restart", command=run_RRT)
button.pack()
top.mainloop()

