# Node class for pathfinding method
__author__ = 'Josh di Bona'


class Node:

    obstacle_bool = False
    obstacle = 0

    # Calculate Heuristic using the Manhattan Method
    H_value = 0 # Heuristic - Distance from node to target node
    # Movements costs will be 10 for any horizontal or vertical movement and a cost of 14 for any diagonal movement
    G_value = 0 # Movement Cost
    F_value = 0 # G + H
    Parent = 0 # A node to reach this node

    def setH(self, value):
        self.H_value = value
    def getH(self):
        return self.H_value

    def setG(self, value):
        self.G_value = value
    def getG(self):
        return self.G_value

    def getF(self):
        value = self.G_value + self.H_value
        return value

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




