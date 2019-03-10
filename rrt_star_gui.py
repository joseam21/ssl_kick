#!/usr/bin/python
import tkinter
import time
import random
import math
import numpy as np
import matplotlib.pyplot as plt

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

class RRT:
    def __init__(self):
        self.start = (200, 300)
        self.goal = (400, 100)
        self.tree_points = [self.start]
        self.obstacles = []
        # draw start and end
        self.generateRandomObstacles(30)
        # print('obstacles', self.obstacles)
        # GUI.drawYellowCircle(self.start[0], self.start[1])
        # GUI.drawYellowCircle(self.goal[0], self.goal[1])
        # for obstacle in self.obstacles:
        #     GUI.drawPinkCircle(obstacle[0], obstacle[1])

    def getRandomPoint(self):
        x = random.randint(0, width)
        y = random.randint(0, height)
        return (x, y)
    
    def generateRandomObstacles(self, num):
        for i in range(num):
            point = self.getRandomPoint()
            self.obstacles.append(point)

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
        # GUI.drawBlueCircle(rand_x, rand_y)
        
        # get closest point
        dists = []
        for point in self.tree_points:
            dist = (point[1] - rand_y)**2 + (point[0] - rand_x)**2
            dists.append(dist)
        ind = np.argmin(np.array(dists))
        closest_neighbor = self.tree_points[ind]
        new_tree_pt = self.point_towards_random((rand_x, rand_y), closest_neighbor)

        # print('new point', new_tree_pt)
        if self.check_obstacles(closest_neighbor, new_tree_pt):
            # GUI.drawBlackCircle(new_tree_pt[0], new_tree_pt[1])
            self.tree_points.append(new_tree_pt)
            #draw point
            # GUI.drawBlackLine(closest_neighbor, new_tree_pt)
            if self.close_to_goal(new_tree_pt):
                return False
        return True
        
    def check_obstacles(self, closest_neighbor, new_tree_pt):
        for obstacle in self.obstacles:
            # print('checking')
            not_in_x = new_tree_pt[0]>obstacle[0]+30 or closest_neighbor[0]<obstacle[0]-30
            # print('not in x', not_in_x)
            not_in_y = new_tree_pt[1]>obstacle[1]+30 or closest_neighbor[1]<obstacle[1]-30
            # print('not in y', not_in_y)
            if not not_in_x and not not_in_y:
                # print('collided obstacle', obstacle)
                return False
        return True

height=500
width=800
# top = tkinter.Tk()
# canvas = tkinter.Canvas(top, bg="green", height=height, width=width)
r = RRT()
new_point = r.start
# while new_point == None or not r.close_to_goal(new_point):
times = []
i=0
while i<20:
    cont = True
    start = time.clock()
    print('start', start)
    while cont:
        cont = r.add_point()
        # print('in loop')
    end = time.clock() - start
    times.append(end)
    print('time',end)
    i+=1
plt.hist(times, bins="auto")
plt.show()
# canvas.pack()
# top.mainloop()
