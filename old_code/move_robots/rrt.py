import pygame
import random
import math
import sys
from Node import Node

def findHeading(p1, p2):
    return math.degrees(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))

pygame.init()
width = 600
height = 600
screen = pygame.display.set_mode((width, height))
done = False

screen.fill((255, 255, 255))
#
image = pygame.image.load("Maze 1.png")
image_rect = image.get_rect()
screen.blit(image, image_rect)
pygame.display.flip()

def validNode(q):
    if q[0] in range(0, width) and q[1] in range(0, height):
        if image.get_at(q) == pygame.Color(0, 0, 0, 255):
            return False
        else:
            return True
    else:
        return False

def endOfTheLine(q, radius):
    for i1 in range(q.getIntCoord()[0] - radius, q.getIntCoord()[0] + radius):
        for j1 in range(q.getIntCoord()[1] - radius, q.getIntCoord()[1] + radius):
            if i1 in range(0, width) and j1 in range(0, height):
                if not validNode((i1, j1)):
                    return True
                    q.setEnd(True)
                # print("hello")

initial_node = Node(40, 40)
# pygame.draw.circle(screen, (255, 0, 0), (50, 50), 10, 1)
nodes = [initial_node]
delta = 10.0
neighborhood = delta * 2.0
node_num = 6000

print("Starting")

for i in range(0, node_num):
    close_nodes = []
    invalid = True
    q_new = Node(random.uniform(0, 1.0) * float(width), random.uniform(0, 1.0) * float(height))

    while invalid:
        if validNode(q_new.getIntCoord()):
            invalid = False
        else:
            q_new = Node(random.uniform(0, 1.0) * float(width), random.uniform(0, 1.0) * float(height))

    q_new.setH(q_new.distance(Node(width-1, height-1)))

    current_node = nodes[0]
    for j in range(0, len(nodes)):
        dist1 = q_new.distance(current_node)
        dist2 = q_new.distance(nodes[j])
        if dist2 < dist1:
            current_node = nodes[j]
    if q_new.distance(current_node) > delta:
        heading = findHeading(current_node.getCoord(), q_new.getCoord())
        q_new = Node(current_node.getCoord()[0]+(delta*math.cos(heading)), current_node.getCoord()[1]+(delta*math.sin(heading)))
        if not validNode(q_new.getIntCoord()):
            continue

    for k in nodes:
        if q_new.distance(k) <= neighborhood:
            close_nodes.append(k)

    current_node = close_nodes[0]
    for l in range(0, len(close_nodes)):
        cost1 = q_new.distance(current_node) + current_node.getCost()
        cost2 = q_new.distance(close_nodes[l]) + close_nodes[l].getCost()
        if cost2 < cost1:
            current_node = close_nodes[l]

    if endOfTheLine(q_new, 8):
        continue

    q_new.setParent(current_node)
    nodes.append(q_new)

    for m in range(0, len(close_nodes)):
        cost1 = close_nodes[m].getCost() + close_nodes[m].getH()
        cost2 = q_new.getCost() + q_new.distance(close_nodes[m]) + q_new.getH()
        if cost2 < cost1:
            close_nodes[m].setParent(q_new)
else:
    current_node = nodes[0]
    close_nodes = []
    goal_node = Node(width-10, height-10)
    for j in range(0, len(nodes)):
        dist1 = goal_node.distance(current_node)
        dist2 = goal_node.distance(nodes[j])
        if dist2 < dist1:
            current_node = nodes[j]
    for k in nodes:
        if current_node.distance(k) <= neighborhood:
            close_nodes.append(k)

    for l in range(0, len(close_nodes)):
        cost1 = goal_node.distance(current_node) + current_node.getCost()
        cost2 = goal_node.distance(close_nodes[l]) + close_nodes[l].getCost()
        if cost2 < cost1:
            current_node = close_nodes[l]

    goal_node.setParent(current_node)
    nodes.append(goal_node)

print("Done 2")

for i in nodes:
    if i.isEnd():
        pygame.draw.circle(screen, (255, 0, 0), i.getIntCoord(), 10, 1)
    else:
        pygame.draw.circle(screen, (255, 0, 0), i.getIntCoord(), 1, 1)
    for j in i.getChildren():
        pygame.draw.lines(screen, (0, 0, 0), False, [i.getIntCoord(), j.getIntCoord()], 1)
        pygame.display.flip()

final = nodes[len(nodes)-1]
stop = False
while not stop:
    if final.getParent() is not False:
        pygame.draw.lines(screen, (255, 0, 255), False, [final.getIntCoord(), final.getParent().getIntCoord()], 2)
        final = final.getParent()
    else:
        stop = True

print("Done 3")

pygame.display.flip()


while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
