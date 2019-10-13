import math, sys

class Node(object):
    def __init__(self, xcor, ycor):
        self.xcor = xcor
        self.ycor = ycor
        
        self.incoming_0 = set()
        self.outgoing_0 = set()
        
        self.incoming_r = set()
        self.outgoing_r = set()
        
        self.g = float('inf')
        self.lmc = float('inf')
        
        self.parent = None
        self.children = set()

    def getCoords(self):
        return (self.xcor, self.ycor)

    def getIntCoords(self):
        return (int(self.xcor), int(self.ycor))

    def distance(self, other):
        other_x, other_y = other.getCoords()
        return math.sqrt((self.xcor - other_x)**2 + (self.ycor - other_y)**2)

    def __lt__(self, other):
        return self

