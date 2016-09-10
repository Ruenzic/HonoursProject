# RobotModule.py Library
__author__ = 'Josh di Bona'

import RPi.GPIO as GPIO
import time
from Node import *


class Robot:

    RIGHT_PWM_PIN = 10
    RIGHT_1_PIN = 10
    RIGHT_2_PIN = 25
    LEFT_PWM_PIN = 17
    LEFT_1_PIN = 17
    LEFT_2_PIN = 4
    SW1_PIN = 11
    SW2_PIN = 9
    LED1_PIN = 8
    LED2_PIN = 7
    OC1_PIN = 22
    OC2_PIN = 27
    OC2_PIN_R1 = 21
    OC2_PIN_R2 = 27
    TRIGGER_PIN = 18
    ECHO_PIN = 23
    left_pwm = 0
    right_pwm = 0
    pwm_scale = 0

    old_left_dir = -1
    old_right_dir = -1

    move_delay = 0.4 #See what happens when you change delay
    voltage = 5
    left_voltage_scale = 0.8
    right_voltage_scale = 0.8

    left_turn_voltage = 0.8
    right_turn_voltage = 0.8

    step_time = 0.5
    step_time_diagonal = 0.707

    turn_time = 0.25

    start_pos = [0,0]
    start_rot = 0
    current_pos = [0,0]
    current_rot = 0

    x_lim = 9 # Bounds for the grid/enclosure. X[0,10]
    y_lim = 9 # Bounds for the grid/enclosure. Y[0,10]

    grid_map = [[0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0]]

    grid_map_clear = [[0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0]]

    def __init__(self,revision=2):

        self.pwm_scale = 1

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.LEFT_PWM_PIN, GPIO.OUT)
        self.left_pwm = GPIO.PWM(self.LEFT_PWM_PIN, 500)
        self.left_pwm.start(0)
        GPIO.setup(self.LEFT_1_PIN, GPIO.OUT)
        GPIO.setup(self.LEFT_2_PIN, GPIO.OUT)

        GPIO.setup(self.RIGHT_PWM_PIN, GPIO.OUT)
        self.right_pwm = GPIO.PWM(self.RIGHT_PWM_PIN, 500)
        self.right_pwm.start(0)
        GPIO.setup(self.RIGHT_1_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_2_PIN, GPIO.OUT)

        GPIO.setup(self.LED1_PIN, GPIO.OUT)
        GPIO.setup(self.LED2_PIN, GPIO.OUT)

        GPIO.setup(self.OC1_PIN, GPIO.OUT)
        if revision == 1:
            self.OC2_PIN = self.OC2_PIN_R1
        else:
            self.OC2_PIN = self.OC2_PIN_R2

        GPIO.setup(self.OC2_PIN_R2, GPIO.OUT)

        GPIO.setup(self.SW1_PIN, GPIO.IN)
        GPIO.setup(self.SW2_PIN, GPIO.IN)
        GPIO.setup(self.TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)

    def set_motors(self, left_pwm, left_dir, right_pwm, right_dir):
        self.set_driver_pins(left_pwm, left_dir, right_pwm, right_dir)
        self.old_left_dir = left_dir
        self.old_right_dir = right_dir

    def set_driver_pins(self, left_pwm, left_dir, right_pwm, right_dir):
        self.left_pwm.ChangeDutyCycle(left_pwm * 100 * self.pwm_scale)
        GPIO.output(self.LEFT_1_PIN, left_dir)
        GPIO.output(self.LEFT_2_PIN, not left_dir)
        self.right_pwm.ChangeDutyCycle(right_pwm * 100 * self.pwm_scale)
        GPIO.output(self.RIGHT_1_PIN, right_dir)
        GPIO.output(self.RIGHT_2_PIN, not right_dir)

    def forward(self, steps = 1): # 1 step by default
        self.stop() # Stop motors before moving them again

        if (self.check_pos(self.current_pos[0], self.current_pos[1], steps)):
            print("Requested move will go out of bounds and therefore can't be performed")
        else:
            print("Moving Robot Forward %d Steps" % steps) # Debugging statement

            # When moving a step forward while at 45 degrees, so rotations 1,5,7,3. You must move root(2) steps forward for every step.
            # Use step_time_diagonal instead of normal step_time
            for num in range(0,steps):
                self.set_motors(self.left_voltage_scale, 1, self.right_voltage_scale, 1)
                if (self.current_rot == 1 or self.current_rot == 5 or self.current_rot == 7 or self.current_rot == 3):
                    time.sleep(self.step_time_diagonal)
                else:
                    time.sleep(self.step_time)
                self.stop()    # Delay between each movement
                time.sleep(self.move_delay)
            self.manage_pos(steps)
            print("Rotation:",self.current_rot) # Debugging
            print("X:",self.current_pos[0])
            print("Y:",self.current_pos[1])

    def stop(self):
        self.set_motors(0, 0, 0, 0)

    def reverse(self, steps = 1): # 1 step by default
        self.stop() # Stop motors before moving them again

        if (self.check_pos(self.current_pos[0], self.current_pos[1], steps * -1)):
            print("Requested move will go out of bounds and therefore can't be performed")
        else:
            print("Moving Robot Backward %d Steps" % steps) # Debugging statement
            for num in range(0,steps):
                self.set_motors(self.left_voltage_scale, 0, self.right_voltage_scale, 0)
                if (self.current_rot == 1 or self.current_rot == 5 or self.current_rot == 7 or self.current_rot == 3):
                    time.sleep(self.step_time_diagonal)
                else:
                    time.sleep(self.step_time)
                self.stop()    # Delay between each movement
                time.sleep(self.move_delay)
            self.manage_pos(steps * -1) # Make the steps negative so that the manage_pos func will move robot in correct dir based on its rot
            print("Rotation:",self.current_rot) # Debugging
            print("X:",self.current_pos[0])
            print("Y:",self.current_pos[1])

    def left(self, steps=1): # 45 degrees by default
        self.stop() # Stop motors before moving them again
        print("Turning Robot Left %d Steps" % steps) # Debugging statement
        if (steps >= 8):
            steps -= 8 # Remove 360 degree turns
        for num in range(0,steps):
            self.set_motors(self.left_turn_voltage, 1, self.right_turn_voltage, 0)
            time.sleep(self.turn_time)
            self.stop()    # Delay between each movement
            time.sleep(self.move_delay)
        self.current_rot -= steps
        self.manage_rot()
        print("Rotation:",self.current_rot) # Debugging
        print("X:",self.current_pos[0])
        print("Y:",self.current_pos[1])

    def right(self, steps=1): # 45 degrees by default
        self.stop() # Stop motors before moving them again
        print("Turning Robot Right %d Steps" % steps) # Debugging statement
        if (steps >= 8):
            steps -= 8 # Remove 360 degree turns
        for num in range(0,steps):
            self.set_motors(self.left_turn_voltage, 0, self.right_turn_voltage, 1)
            time.sleep(self.turn_time)
            self.stop()    # Delay between each movement
            time.sleep(self.move_delay)
        self.current_rot += steps
        self.manage_rot()
        print("Rotation:",self.current_rot) # Debugging
        print("X:",self.current_pos[0])
        print("Y:",self.current_pos[1])

    def set_led1(self, state): # 0/1 for state
        GPIO.output(self.LED1_PIN, state)

    def set_led2(self, state): # 0/1 for state
        GPIO.output(self.LED2_PIN, state)

    def cleanup(self):
        GPIO.cleanup()

    def manage_rot(self): # Manage the current rotation by removing 360 degrees when needed.
        if (self.current_rot >= 8):
            self.current_rot -= 8
        elif (self.current_rot < 0):
            self.current_rot += 8


    def manage_pos(self, steps): # Method to manage the position of the robot in the grid world (keep track)

        self.manage_rot() # Make sure rotation is in positive form for ease

        if (self.current_rot == 0): # Increase the y value by the number of steps
            self.current_pos[1] += steps
        elif (self.current_rot == 1): # Increase x and y
            self.current_pos[0] += steps
            self.current_pos[1] += steps
        elif (self.current_rot == 2): # Increase x
            self.current_pos[0] += steps
        elif (self.current_rot == 3): # Increase x, Decrease y
            self.current_pos[0] += steps
            self.current_pos[1] -= steps
        elif (self.current_rot == 4): # Decrease y
            self.current_pos[1] -= steps
        elif (self.current_rot == 5): # Decrease x and y
            self.current_pos[0] -= steps
            self.current_pos[1] -= steps
        elif (self.current_rot == 6): # Decrease x
            self.current_pos[0] -= steps
        elif (self.current_rot == 7): # Decrease x, Increase y
            self.current_pos[0] -= steps
            self.current_pos[1] += steps

    def check_pos(self, x, y, steps): # Method to check if moving the robot will move it out of bounds.

        self.manage_rot() # Make sure rotation is in positive form for ease

        if (self.current_rot == 0): # Increase the y value by the number of steps
            y += steps
        elif (self.current_rot == 1): # Increase x and y
            x += steps
            y += steps
        elif (self.current_rot == 2): # Increase x
            x += steps
        elif (self.current_rot == 3): # Increase x, Decrease y
            x += steps
            y -= steps
        elif (self.current_rot == 4): # Decrease y
            y -= steps
        elif (self.current_rot == 5): # Decrease x and y
            x -= steps
            y -= steps
        elif (self.current_rot == 6): # Decrease x
            x -= steps
        elif (self.current_rot == 7): # Decrease x, Increase y
            x -= steps
            y += steps

        if (x > self.x_lim or x < 0 or y > self.y_lim or y < 0):
            return True # The next move violates the boundaries
        else:
            return False # The next move won't violate the boundaries

    def reset(self):
        print("Resetting Position of Robot")
        # Reverse if the robot has the correct opposite rotation that it needs to be in
        # [TODO] Move diagonally?

        # Move the robot back to the start position
        # Move on the y axis

        # Check to see if there are obstacles present, if not, use this method, else call pathfinding and follow the path.

        if (self.grid_map == self.grid_map_clear): # No obstacles

            if (self.current_pos[1] > 0):
                if (self.current_rot != 4 and self.current_rot != 0):
                    # rotate towards the start
                    if (self.current_rot >= 0 and self.current_rot < 4):
                        self.right(4 - self.current_rot)
                    elif (self.current_rot > 4):
                        self.left(self.current_rot - 4)
                if (self.current_rot == 4):
                    self.forward(self.current_pos[1])
                elif (self.current_rot == 0):
                    self.reverse(self.current_pos[1])

            elif (self.current_pos[1] < 0):
                if (self.current_rot != 0 and self.current_rot != 4):
                    # rotate towards the start
                    if (self.current_rot >= 0 and self.current_rot < 4):
                        self.left(self.current_rot)
                    elif (self.current_rot > 4):
                        self.right(8 - self.current_rot)
                if (self.current_rot == 0):
                    self.forward(self.current_pos[1] * -1) # Change negative to positive
                elif (self.current_rot == 4):
                    self.reverse(self.current_pos[1] * -1)

            # Move on the x axis
            if (self.current_pos[0] > 0):
                if (self.current_rot != 6 and self.current_rot != 2):
                    # rotate towards the start
                    if (self.current_rot >= 2 and self.current_rot < 6):
                        self.right(6 - self.current_rot)
                    elif (self.current_rot < 2):
                        self.left(2 + self.current_rot)
                    elif (self.current_rot > 6):
                        self.left(7 - self.current_rot)
                if (self.current_rot == 6):
                    self.forward(self.current_pos[0])
                elif (self.current_rot == 2):
                    self.reverse(self.current_pos[0])

            elif (self.current_pos[0] < 0):
                if (self.current_rot != 2 and self.current_rot != 6):
                    # rotate towards the start
                    if (self.current_rot >= 2 and self.current_rot < 6):
                        self.left(self.current_rot - 2)
                    elif (self.current_rot < 2):
                        self.right(2 - self.current_rot)
                    elif (self.current_rot > 6):
                        self.right(8 - self.current_rot + 1)
                if (self.current_rot == 2):
                    self.forward(self.current_pos[0] * -1) # Change the negative to positive
                elif (self.current_rot == 6):
                    self.reverse(self.currrent_pos[0] * -1)


            # Set the rotation of the robot back to the original direction
            # Check to see if rotation is bigger or smaller than 4, then rotate the shorter direction
            if (self.current_rot != 0):
                # rotate towards the start
                if (self.current_rot >= 0 and self.current_rot < 4):
                    self.left(self.current_rot)
                elif (self.current_rot >= 4):
                    self.right(8 - self.current_rot)

        else:
            self.follow_path(self.pathfind(self.get_pos(),self.get_start_pos()))
            if (self.get_rot() != self.get_start_rot()):
                self.change_rot(self.get_rot(),self.get_start_rot())

        self.current_pos[0] = 0
        self.current_pos[1] = 0
        self.current_rot = 0

    #[TODO] Allow the use of this for when students are with the robot and can reset the robot manually after.
    #[TODO] Use Jeremy's sensor to check if about to hit an obstacle, if so stop motors, in the contForward and Reverse

    def contForward(self): # Move the robot forward continuously with no sleep or delay
        self.stop() # Stop motors before moving them again
        print("Moving Robot Forward Indefinitely")
        self.set_motors(self.left_voltage_scale, 1, self.right_voltage_scale, 1)

    def contReverse(self): # Move the robot backwards continuously with no sleep or delay
        self.stop() # Stop motors before moving them again
        print("Moving Robot Backward Indefinitely")
        self.set_motors(self.left_voltage_scale, 0, self.right_voltage_scale, 0)

    def contLeft(self): # Turn the robot left continuously with no sleep or delay
        self.stop() # Stop motors before moving them again
        print("Turing Robot Left Continuously")
        self.set_motors(self.left_turn_voltage, 1, self.right_turn_voltage, 0)

    def contRight(self): # Turn the robot left continuously with no sleep or delay
        self.stop() # Stop motors before moving them again
        print("Turing Robot Right Continuously")
        self.set_motors(self.left_turn_voltage, 0, self.right_turn_voltage, 1)

    def follow_path(self, nodes_path): # Method that will have the robot follow a path returned by pathfinding
        print ("Robot Following A* Path:")
        # Loop through nodes_path backwards, as it is from goal - > start
        # At each node, check where the parent is, if above, below, left, right or any diagonal
        # After check what the current rotation of the robot is
        # Change rotation, move robot

        for i in range (len(nodes_path)-1, 0):
            if (i != 0):
                current_node = nodes_path[i]
                next_node = nodes_path[i - 1]
                current_pos = current_node.getPosition()
                next_pos = next_node.getPosition()
                current_rot = self.get_rot()
                print("node " + str(i))
                rot1 = 0
                rot2 = 0

                if (next_pos[0] > current_pos[0] and next_pos[1] == current_pos[1]): # Right
                    rot1 = 2
                    rot2 = 6
                elif (next_pos[0] < current_pos[0] and next_pos[1] == current_pos[1]): # Left
                    rot1 = 6
                    rot2 = 2
                elif (next_pos[0] == current_pos[0] and next_pos[1] > current_pos[1]): # Above
                    rot1 = 0
                    rot2 = 4
                elif (next_pos[0] == current_pos[0] and next_pos[1] < current_pos[1]): # Below
                    rot1 = 4
                    rot2 = 0
                elif (next_pos[0] > current_pos[0] and next_pos[1] > current_pos[1]): # Top right
                    rot1 = 1
                    rot2 = 5
                elif (next_pos[0] < current_pos[0] and next_pos[1] < current_pos[1]): # Bottom left
                    rot1 = 5
                    rot2 = 1
                elif (next_pos[0] > current_pos[0] and next_pos[1] < current_pos[1]): # Bottom right
                    rot1 = 3
                    rot2 = 7
                elif (next_pos[0] < current_pos[0] and next_pos[1] > current_pos[1]): # Top left
                    rot1 = 7
                    rot2 = 3

                # Now that we know the position of the next spot, change rotation and move there

                if (current_rot != rot1 and current_rot != rot2):
                    print("changing rot, from " + str(current_rot) + " to " + str(rot1))
                    self.change_rot(current_rot, rot1)
                if (current_rot == rot1):
                    self.forward(1)
                elif (current_rot == rot2):
                    self.reverse(1)

    def change_rot(self, current_rot, goal_rot): # Method to change rotation from current to destination rotation
        # Check to see if its quicker to rotate left or right to destination rotation
        # Turn left or right the required steps

        leftTurn = 0
        rightTurn = 0

        if (goal_rot > current_rot):
            rightTurn = abs(goal_rot - current_rot)
            leftTurn = abs(8 - goal_rot + current_rot)
        elif (goal_rot < current_rot):
            rightTurn = abs(8 - current_rot + goal_rot)
            leftTurn = abs(current_rot - goal_rot)

        if (leftTurn < rightTurn):
            self.left(leftTurn)
        else:
            self.right(rightTurn)

    def get_pos(self): # Return [x,y] position of robot to the user
        return self.current_pos

    def get_rot(self): # Return rotation variable to user
        return self.current_rot

    def add_obstacles(self, obstacles):
        num = len(obstacles)
        for i in range (0,num):
            x = obstacles[i][0]
            y = obstacles[i][1]
            self.grid_map[y][x] = 1

    def reset_obstacles(self):
        for i in range (0,self.y_lim + 1):
            for j in range (0, self.x_lim + 1):
                self.grid_map[i][j] = 0


    def pathfind(self, start_pos, goal_pos):
        print("Running Pathfinding Algorithm")

        grid_nodes = []

        # Create the grid of nodes
        for i in range (0,self.y_lim + 1):
            temp = []
            for j in range (0, self.x_lim + 1):
                state = self.grid_map[i][j]
                tempNode = Node(state, j, i)
                #tempNode.setObstacle(state)
                temp.append(tempNode)
            grid_nodes.append(temp)

        start_node = grid_nodes[start_pos[1]][start_pos[0]] # Changed from grid_map to grid_nodes
        goal_node = grid_nodes[goal_pos[1]][goal_pos[0]]

        closed_set = [] # Set of nodes that have been checked
        open_set = [start_node] # Set of nodes that need to be checked

        #Calculate the G values of all nodes in grid_nodes
        for i in range (0,self.y_lim + 1):
            for j in range (0, self.x_lim + 1):
                node = grid_nodes[i][j]
                G_value = abs(goal_pos[0] - j) + abs(goal_pos[1] - i)
                node.setG(abs(G_value))
                #print(j,i,"    ", grid_nodes[i][j].getG()) # Debugging

        # While open set is not empty
        # Take next element out of openset with lowest F score
        # Check if this node is the goal node
        # Remove it from the set
        # Add it to closed set
        # For each of the nodes neighbours
                # if not closed, if not in open, add to open.
                # if current g score + distance between the 2 is greater, ignore

        # Note: G cost = Parent G cost + movement cost (10/14)


        while (len(open_set) > 0 ):
            # Use smallest F value node next in our list
            current_node = self.find_smallest(open_set)
            # Add to closed set 'mark closed'
            current_node.close()
            # Remove from open set
            open_set.remove(current_node)
            # Add neighbours if they aren't added already
            self.add_neighbours(grid_nodes, open_set, current_node)

        # Once the open list is empty, create path
        nodes_path = []
        current = goal_node
        while (current != start_node):
            nodes_path.append(current)
            #print(current)
            temp = current.getParent()
            current = temp

        # Print path to user
        print ("Path for robot found:")
        for i in range (self.y_lim,-1,-1):
            line = ""
            for j in range (0, self.x_lim+1):
                if (grid_nodes[i][j] == start_node):
                    line += " S "
                elif (grid_nodes[i][j] == goal_node):
                    line += " E "
                elif (grid_nodes[i][j] in nodes_path):
                    line += " 0 "
                elif (grid_nodes[i][j].getObstacle() == True):
                    line += " X "
                else:
                    line += " - "
            print(line)

        return nodes_path


    def add_neighbours(self, grid_nodes, open_set, current_node): # Add the current nodes neighbours to the open_set
        current_pos = current_node.getPosition()
        if (current_pos[0] + 1 <= self.x_lim):
            node = grid_nodes[current_pos[1]][current_pos[0] + 1] # Right
            self.test_node(node, open_set, current_node, 10)

        if (current_pos[0] - 1 >= 0):
            node = grid_nodes[current_pos[1]][current_pos[0] - 1] # Left
            self.test_node(node, open_set, current_node, 10)

        if (current_pos[1] + 1 <= self.y_lim):
            node = grid_nodes[current_pos[1] + 1][current_pos[0]] # Above
            self.test_node(node, open_set, current_node, 10)

        if (current_pos[1] - 1 >= 0):
            node = grid_nodes[current_pos[1] - 1][current_pos[0]] # Below
            self.test_node(node, open_set, current_node, 10)


        if (current_pos[0] + 1 <= self.x_lim and current_pos[1] + 1 < self.y_lim):
            node = grid_nodes[current_pos[1] + 1][current_pos[0] + 1] # Top Right
            self.test_node(node, open_set, current_node, 14)

        if (current_pos[0] - 1 >= 0 and current_pos[1] + 1 < self.y_lim):
            node = grid_nodes[current_pos[1] + 1][current_pos[0] - 1] # Top Left
            self.test_node(node, open_set, current_node, 14)

        if (current_pos[0] + 1 <= self.x_lim and current_pos[1] - 1 > 0):
            node = grid_nodes[current_pos[1] - 1][current_pos[0] + 1] # Bottom Right
            self.test_node(node, open_set, current_node, 14)

        if (current_pos[0] - 1 >= 0 and current_pos[1] - 1 > 0):
            node = grid_nodes[current_pos[1] - 1][current_pos[0] - 1] # Bottom Left
            self.test_node(node, open_set, current_node, 14)


    def test_node(self, node, open_set, current_node, cost):
        if (node.isclosed() == False and node.getObstacle() == False):
            if (node not in open_set):
                open_set.append(node) # Right
                node.setG(current_node.getG() + cost)
                node.calcF()
                node.setParent(current_node)
            elif (current_node.getG() + cost < node.getG()):
                node.setParent(current_node)


    def find_smallest(self, open_set):
        if (len(open_set) == 1):
            return open_set[0]
        smallest = 0
        for i in range (0,len(open_set)):
            if (i == 0):
                smallest = open_set[i]
            else:
                if (open_set[i].getF() < smallest.getF()):
                    smallest = open_set[i]
        return smallest




    def set_start_pos(self, x, y, rotation): # Method to set the x and y start position of the robot, as well as the rotation
        self.start_pos[0] = x
        self.start_pos[1] = y
        self.start_rot = rotation

    def get_start_pos(self): # Method to return the start position
        return self.start_pos

    def get_start_rot(self): # Method to return the start rotation
        return self.start_rot


    def print_grid(self): # Print out the grid, with 0 meaning no obstacle and 1 an obstacle
        for i in range (self.y_lim,-1,-1):
            print(self.grid_map[i])
































