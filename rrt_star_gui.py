#!/usr/bin/python
import tkinter
import random
import math
import numpy as np

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

class RRT:
    def __init__(self):
        self.start = (200, 300)
        self.goal = (400, 100)
        self.tree_points = [self.start]
        # draw start and end
        GUI.drawYellowCircle(self.start[0], self.start[1])
        GUI.drawYellowCircle(self.goal[0], self.goal[1])

    def getRandomPoint(self):
        x = random.randint(0, width)
        y = random.randint(0, height)
        return (x, y)

    def point_towards_random(self, random_pt, tree_pt, d = 10):
        (x1, y1) = tree_pt
        (x2, y2) = random_pt
        theta = math.atan2((y2 - y1), (x2 - x1))
        x_d = round(x1 + d*math.cos(theta))
        y_d = round(y1 + d*math.sin(theta))
        return x_d, y_d

    def close_to_goal(self, pt):
        dist = ((pt[1] - self.goal[1])**2 + (pt[0] - self.goal[0])**2)**0.5
        return dist < 10
    
    def add_point(self):
        rand_x, rand_y = self.getRandomPoint()
        GUI.drawBlueCircle(rand_x, rand_y)
        
        # get closest point
        dists = []
        for point in self.tree_points:
            dist = (point[1] - rand_y)**2 + (point[0] - rand_x)**2
            dists.append(dist)
        ind = np.argmin(np.array(dists))
        closest_neighbor = self.tree_points[ind]

        new_tree_pt = self.point_towards_random((rand_x, rand_y), closest_neighbor)
        GUI.drawBlackCircle(new_tree_pt[0], new_tree_pt[1])
        self.tree_points.append(new_tree_pt)
        #draw point
        GUI.drawBlackLine(closest_neighbor, new_tree_pt)
        return new_tree_pt
        
        

height=500
width=800
top = tkinter.Tk()
canvas = tkinter.Canvas(top, bg="green", height=height, width=width)
r = RRT()
new_point = r.start
while not r.close_to_goal(new_point):
    new_point = r.add_point()

canvas.pack()
top.mainloop()
