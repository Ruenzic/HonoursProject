# Node class for pathfinding method
__author__ = 'Josh di Bona'


class Node:

    x = 0
    y = 0
    position = []

    closed = False
    obstacle_bool = False
    obstacle = 0

    # Calculate Heuristic using the Manhattan Method
    H_value = 0 # Heuristic - Distance from node to target node
    # Movements costs will be 10 for any horizontal or vertical movement and a cost of 14 for any diagonal movement
    G_value = 0 # Movement Cost
    F_value = 0 # G + H
    Parent = 5 # A node to reach this node

    def setH(self, value):
        self.H_value = value
    def getH(self):
        return self.H_value

    def setG(self, value):
        self.G_value = value
    def getG(self):
        return self.G_value

    def getF(self):
       return self.F_value

    def calcF(self):
        self.F_value = self.G_value + self.H_value

    def setParent(self, parent):
        self.Parent = parent
    def getParent(self):
        return self.Parent

    def setObstacle(self, state):
        self.obstacle = state
        self.checkState()

    def checkState(self):
        if (self.obstacle == 0):
            self.obstacle_bool = False
        else:
            self.obstacle_bool = True

    def getObstacle(self):
        return self.obstacle_bool

    def close(self):
        self.closed = True

    def isclosed(self):
        return self.closed

    def getPosition(self):
        return self.position

    def __init__(self, state, x_ord, y_ord):
        self.setObstacle(state)
        self.x = x_ord
        self.y = y_ord
        self.position = [x_ord, y_ord]


