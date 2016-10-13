# RobotModule.py Library
__author__ = 'Josh di Bona'

import RPi.GPIO as GPIO
import time
from Node import *


class _Singleton(type):
    """ A metaclass that creates a Singleton base class when called. """
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(_Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class Singleton(_Singleton('SingletonMeta', (object,), {})): pass


class Movement(Singleton):
    pass


    __RIGHT_PWM_PIN = 19 # 10
    __RIGHT_1_PIN = 19 # 10
    __RIGHT_2_PIN = 22 # 25
    __LEFT_PWM_PIN = 11 # 17
    __LEFT_1_PIN = 11 # 17
    __LEFT_2_PIN = 7 # 4
    __LED1_PIN = 24 # 8
    __LED2_PIN = 26 # 7
    __left_pwm = 0
    __right_pwm = 0
    __pwm_scale = 0

    __old_left_dir = -1
    __old_right_dir = -1

    __move_delay = 0.4 #See what happens when you change delay
    __voltage = 5
    __left_voltage_scale_reverse = 0.66
    __right_voltage_scale_reverse = 0.7

    __left_voltage_scale_forward = 0.65
    __right_voltage_scale_forward = 0.735

    # Make a forwards and reverse sleep timers, different
    # For forward 1.0
    # 1.3 for reverse

    # will need to make a new reverse diagonal

    __left_turn_voltage = 0.8
    __right_turn_voltage = 0.8

    __step_time = 1.15
    __step_time_diagonal = 1.6

    __turn_time = 0.335

    start_pos = [2,3]
    start_rot = 0
    __current_pos = [2,3]
    __current_rot = 0

    __x_lim = 5 # Bounds for the grid/enclosure. X[0,10]
    __y_lim = 7 # Bounds for the grid/enclosure. Y[0,10]

    grid_map = [[0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
		[0,0,0,0,0,0]]

    grid_map_clear = [[0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
		[0,0,0,0,0,0]]

    def __init__(self):

        self.__pwm_scale = 1

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(self.__LEFT_PWM_PIN, GPIO.OUT)
        self.__left_pwm = GPIO.PWM(self.__LEFT_PWM_PIN, 500)
        self.__left_pwm.start(0)
        GPIO.setup(self.__LEFT_1_PIN, GPIO.OUT)
        GPIO.setup(self.__LEFT_2_PIN, GPIO.OUT)

        GPIO.setup(self.__RIGHT_PWM_PIN, GPIO.OUT)
        self.__right_pwm = GPIO.PWM(self.__RIGHT_PWM_PIN, 500)
        self.__right_pwm.start(0)
        GPIO.setup(self.__RIGHT_1_PIN, GPIO.OUT)
        GPIO.setup(self.__RIGHT_2_PIN, GPIO.OUT)

        GPIO.setup(self.__LED1_PIN, GPIO.OUT)
        GPIO.setup(self.__LED2_PIN, GPIO.OUT)


    def __set_motors(self, __left_pwm, left_dir, __right_pwm, right_dir):
        self.__set_driver_pins(__left_pwm, left_dir, __right_pwm, right_dir)
        self.__old_left_dir = left_dir
        self.__old_right_dir = right_dir

    def __set_driver_pins(self, __left_pwm, left_dir, __right_pwm, right_dir):
        self.__left_pwm.ChangeDutyCycle(__left_pwm * 100 * self.__pwm_scale)
        GPIO.output(self.__LEFT_1_PIN, left_dir)
        GPIO.output(self.__LEFT_2_PIN, not left_dir)
        self.__right_pwm.ChangeDutyCycle(__right_pwm * 100 * self.__pwm_scale)
        GPIO.output(self.__RIGHT_1_PIN, right_dir)
        GPIO.output(self.__RIGHT_2_PIN, not right_dir)

    def forward(self, steps = 1): # 1 step by default
        self.stop() # Stop motors before moving them again

        if (self.__check_pos(self.__current_pos[0], self.__current_pos[1], steps)):
            print("Requested move will either go out of bounds or hit an obstacle and therefore can't be performed")
        else:
            print("Moving Robot Forward %d Steps" % steps) # Debugging statement

            # When moving a step forward while at 45 degrees, so rotations 1,5,7,3. You must move root(2) steps forward for every step.
            # Use step_time_diagonal instead of normal step_time
            for num in range(0,steps):
                self.__set_motors(self.__left_voltage_scale_forward, 1, self.__right_voltage_scale_forward, 1)
                if (self.__current_rot == 1 or self.__current_rot == 5 or self.__current_rot == 7 or self.__current_rot == 3):
                    time.sleep(self.__step_time_diagonal)
                else:
                    time.sleep(self.__step_time)
                self.stop()    # Delay between each movement
                time.sleep(self.__move_delay)
            self.__manage_pos(steps)
            print("Rotation:",self.__current_rot) # Debugging
            print("X:",self.__current_pos[0])
            print("Y:",self.__current_pos[1])

    def stop(self):
        self.__set_motors(0, 0, 0, 0)

    def reverse(self, steps = 1): # 1 step by default
        self.stop() # Stop motors before moving them again

        if (self.__check_pos(self.__current_pos[0], self.__current_pos[1], steps * -1)):
            print("Requested move will either go out of bounds or hit an obstacle and therefore can't be performed")
        else:
            print("Moving Robot Backward %d Steps" % steps) # Debugging statement
            for num in range(0,steps):
                self.__set_motors(self.__left_voltage_scale_reverse, 0, self.__right_voltage_scale_reverse, 0)
                if (self.__current_rot == 1 or self.__current_rot == 5 or self.__current_rot == 7 or self.__current_rot == 3):
                    time.sleep(self.__step_time_diagonal)
                else:
                    time.sleep(self.__step_time)
                self.stop()    # Delay between each movement
                time.sleep(self.__move_delay)
            self.__manage_pos(steps * -1) # Make the steps negative so that the manage_pos func will move robot in correct dir based on its rot
            print("Rotation:",self.__current_rot) # Debugging
            print("X:",self.__current_pos[0])
            print("Y:",self.__current_pos[1])

    def left(self, steps=1): # 45 degrees by default
        self.stop() # Stop motors before moving them again
        print("Turning Robot Left %d Steps" % steps) # Debugging statement
        if (steps >= 8):
            steps -= 8 # Remove 360 degree turns
        for num in range(0,steps):
            self.__set_motors(self.__left_turn_voltage, 0, self.__right_turn_voltage, 1)
            time.sleep(self.__turn_time)
            self.stop()    # Delay between each movement
            time.sleep(self.__move_delay)
        self.__current_rot -= steps
        self.__manage_rot()
        print("Rotation:",self.__current_rot) # Debugging
        print("X:",self.__current_pos[0])
        print("Y:",self.__current_pos[1])

    def right(self, steps=1): # 45 degrees by default
        self.stop() # Stop motors before moving them again
        print("Turning Robot Right %d Steps" % steps) # Debugging statement
        if (steps >= 8):
            steps -= 8 # Remove 360 degree turns
        for num in range(0,steps):
            self.__set_motors(self.__left_turn_voltage, 1, self.__right_turn_voltage, 0)
            time.sleep(self.__turn_time)
            self.stop()    # Delay between each movement
            time.sleep(self.__move_delay)
        self.__current_rot += steps
        self.__manage_rot()
        print("Rotation:",self.__current_rot) # Debugging
        print("X:",self.__current_pos[0])
        print("Y:",self.__current_pos[1])

    def set_led_red(self, state): # 0/1 for state
        GPIO.output(self.__LED1_PIN, state)

    def set_led_green(self, state): # 0/1 for state
        GPIO.output(self.__LED2_PIN, state)

    def cleanup(self):
        GPIO.cleanup()

    def __manage_rot(self): # Manage the current rotation by removing 360 degrees when needed.
        if (self.__current_rot >= 8):
            self.__current_rot -= 8
        elif (self.__current_rot < 0):
            self.__current_rot += 8


    def __manage_pos(self, steps): # Method to manage the position of the robot in the grid world (keep track)

        self.__manage_rot() # Make sure rotation is in positive form for ease

        if (self.__current_rot == 0): # Increase the y value by the number of steps
            self.__current_pos[1] += steps
        elif (self.__current_rot == 1): # Increase x and y
            self.__current_pos[0] += steps
            self.__current_pos[1] += steps
        elif (self.__current_rot == 2): # Increase x
            self.__current_pos[0] += steps
        elif (self.__current_rot == 3): # Increase x, Decrease y
            self.__current_pos[0] += steps
            self.__current_pos[1] -= steps
        elif (self.__current_rot == 4): # Decrease y
            self.__current_pos[1] -= steps
        elif (self.__current_rot == 5): # Decrease x and y
            self.__current_pos[0] -= steps
            self.__current_pos[1] -= steps
        elif (self.__current_rot == 6): # Decrease x
            self.__current_pos[0] -= steps
        elif (self.__current_rot == 7): # Decrease x, Increase y
            self.__current_pos[0] -= steps
            self.__current_pos[1] += steps

    def __check_pos(self, x, y, steps): # Method to check if moving the robot will move it out of bounds.

        self.__manage_rot() # Make sure rotation is in positive form for ease

        if (self.__current_rot == 0): # Increase the y value by the number of steps
            y += steps
        elif (self.__current_rot == 1): # Increase x and y
            x += steps
            y += steps
        elif (self.__current_rot == 2): # Increase x
            x += steps
        elif (self.__current_rot == 3): # Increase x, Decrease y
            x += steps
            y -= steps
        elif (self.__current_rot == 4): # Decrease y
            y -= steps
        elif (self.__current_rot == 5): # Decrease x and y
            x -= steps
            y -= steps
        elif (self.__current_rot == 6): # Decrease x
            x -= steps
        elif (self.__current_rot == 7): # Decrease x, Increase y
            x -= steps
            y += steps

        if (x > self.__x_lim or x < 0 or y > self.__y_lim or y < 0 or self.grid_map[y][x] == 1):
            return True # The next move violates the boundaries
        else:
            return False # The next move won't violate the boundaries

    def reset_position(self):
        print("Resetting Position of Robot")
        # Reverse if the robot has the correct opposite rotation that it needs to be in
        # [TODO] Move diagonally?

        # Move the robot back to the start position
        # Move on the y axis

        # Check to see if there are obstacles present, if not, use this method, else call pathfinding and follow the path.

        if (self.grid_map == self.grid_map_clear): # No obstacles
            startx = self.get_start_pos()[0]
            starty = self.get_start_pos()[1]
            startrot = self.get_start_rot()

            if (self.__current_pos[1] > starty):
                if (self.__current_rot != 4 and self.__current_rot != 0):
                    # rotate towards the start
                    if (self.__current_rot >= 0 and self.__current_rot < 4):
                        self.right(4 - self.__current_rot)
                    elif (self.__current_rot > 4):
                        self.left(self.__current_rot - 4)
                if (self.__current_rot == 4):
                    self.forward(self.__current_pos[1] - starty)
                elif (self.__current_rot == 0):
                    self.reverse(self.__current_pos[1] - starty)

            elif (self.__current_pos[1] < starty):
                if (self.__current_rot != 0 and self.__current_rot != 4):
                    # rotate towards the start
                    if (self.__current_rot >= 0 and self.__current_rot < 4):
                        self.left(self.__current_rot)
                    elif (self.__current_rot > 4):
                        self.right(8 - self.__current_rot)
                if (self.__current_rot == 0):
                    self.forward(starty - self.__current_pos[1]) # Change negative to positive
                elif (self.__current_rot == 4):
                    self.reverse(starty - self.__current_pos[1])

            # Move on the x axis
            if (self.__current_pos[0] > startx):
                if (self.__current_rot != 6 and self.__current_rot != 2):
                    # rotate towards the start
                    if (self.__current_rot >= 2 and self.__current_rot < 6):
                        self.right(6 - self.__current_rot)
                    elif (self.__current_rot < 2):
                        self.left(2 + self.__current_rot)
                    elif (self.__current_rot > 6):
                        self.left(7 - self.__current_rot)
                if (self.__current_rot == 6):
                    self.forward(self.__current_pos[0] - startx)
                elif (self.__current_rot == 2):
                    self.reverse(self.__current_pos[0] - startx)

            elif (self.__current_pos[0] < startx):
                if (self.__current_rot != 2 and self.__current_rot != 6):
                    # rotate towards the start
                    if (self.__current_rot >= 2 and self.__current_rot < 6):
                        self.left(self.__current_rot - 2)
                    elif (self.__current_rot < 2):
                        self.right(2 - self.__current_rot)
                    elif (self.__current_rot > 6):
                        self.right(8 - self.__current_rot + 1)
                if (self.__current_rot == 2):
                    self.forward(startx - self.__current_pos[0]) # Change the negative to positive
                elif (self.__current_rot == 6):
                    self.reverse(startx - self.currrent_pos[0])


            # Set the rotation of the robot back to the original direction
            # Check to see if rotation is bigger or smaller than 4, then rotate the shorter direction
            if (self.__current_rot != startrot):
                # rotate towards the start
                self.change_rot(self.__current_rot, startrot)

        else:
            self.follow_path(self.pathfind(self.get_pos(),self.get_start_pos()))
            if (self.get_rot() != self.get_start_rot()):
                self.change_rot(self.get_rot(),self.get_start_rot())

        self.__current_pos[0] = self.start_pos[0]
        self.__current_pos[1] = self.start_pos[1]
        self.__current_rot = self.start_rot

    #[TODO] Allow the use of this for when students are with the robot and can reset the robot manually after.
    #[TODO] Use Jeremy's sensor to check if about to hit an obstacle, if so stop motors, in the contForward and Reverse

    def cont_forward(self): # Move the robot forward continuously with no sleep or delay
        self.stop() # Stop motors before moving them again
        print("Moving Robot Forward Indefinitely")
        self.__set_motors(self.__left_voltage_scale_forward, 1, self.__right_voltage_scale_forward, 1)

    def cont_reverse(self): # Move the robot backwards continuously with no sleep or delay
        self.stop() # Stop motors before moving them again
        print("Moving Robot Backward Indefinitely")
        self.__set_motors(self.__left_voltage_scale_reverse, 0, self.__right_voltage_scale_reverse, 0)

    def cont_left(self): # Turn the robot left continuously with no sleep or delay
        self.stop() # Stop motors before moving them again
        print("Turing Robot Left Continuously")
        self.__set_motors(self.__left_turn_voltage, 0, self.__right_turn_voltage, 1)

    def cont_right(self): # Turn the robot left continuously with no sleep or delay
        self.stop() # Stop motors before moving them again
        print("Turing Robot Right Continuously")
        self.__set_motors(self.__left_turn_voltage, 1, self.__right_turn_voltage, 0)

    def follow_path(self, nodes_path): # Method that will have the robot follow a path returned by pathfinding

        if (nodes_path == "No Path"):
            print("No Path can be found with current obstacle setup")
        else:

            print ("Robot Following A* Path:")
            # Loop through nodes_path backwards, as it is from goal - > start
            # At each node, check where the parent is, if above, below, left, right or any diagonal
            # After check what the current rotation of the robot is
            # Change rotation, move robot
            #print("length of nodes path is " + str(len(nodes_path))) # Note, prints out 4
            for i in range (len(nodes_path)-1, 0, -1): # Added -1 at end
                if (i != 0):
                    current_node = nodes_path[i]
                    next_node = nodes_path[i - 1]
                    current_pos = current_node.getPosition()
                    next_pos = next_node.getPosition()
                    current_rot = self.get_rot()
                    #print("node " + str(i)) # Note, follow_path doesnt even get to here
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
                    if (self.get_rot() == rot1):
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
        return self.__current_pos

    def get_rot(self): # Return rotation variable to user
        return self.__current_rot

    def add_obstacles(self, obstacles):
        num = len(obstacles)
        for i in range (0,num):
            x = obstacles[i][0]
            y = obstacles[i][1]
            self.grid_map[y][x] = 1

    def reset_obstacles(self):
        for i in range (0,self.__y_lim + 1):
            for j in range (0, self.__x_lim + 1):
                self.grid_map[i][j] = 0


    def pathfind(self, start_pos, goal_pos):
        print("Running Pathfinding Algorithm")

        grid_nodes = []

        # Create the grid of nodes
        for i in range (0,self.__y_lim + 1):
            temp = []
            for j in range (0, self.__x_lim + 1):
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
        for i in range (0,self.__y_lim + 1):
            for j in range (0, self.__x_lim + 1):
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
            current_node = self.__find_smallest(open_set)
            # Add to closed set 'mark closed'
            current_node.close()
            # Remove from open set
            open_set.remove(current_node)
            # Add neighbours if they aren't added already
            self.__add_neighbours(grid_nodes, open_set, current_node)

        # Once the open list is empty, create path
        nodes_path = []
        current = goal_node


        while (current != start_node):
            try:
                nodes_path.append(current)
                #print(current)
                temp = current.getParent()
                current = temp
            except TypeError and AttributeError: # If goal node has no parent as it cant find a path
                return "No Path"
        nodes_path.append(start_node)
        # Print path to user
        print ("Path for robot found:")
        for i in range (self.__y_lim,-1,-1):
            line = ""
            for j in range (0, self.__x_lim+1):
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

        #print(len(nodes_path))
        return nodes_path


    def __add_neighbours(self, grid_nodes, open_set, current_node): # Add the current nodes neighbours to the open_set
        current_pos = current_node.getPosition()
        if (current_pos[0] + 1 <= self.__x_lim):
            node = grid_nodes[current_pos[1]][current_pos[0] + 1] # Right
            self.__test_node(node, open_set, current_node, 10)

        if (current_pos[0] - 1 >= 0):
            node = grid_nodes[current_pos[1]][current_pos[0] - 1] # Left
            self.__test_node(node, open_set, current_node, 10)

        if (current_pos[1] + 1 <= self.__y_lim):
            node = grid_nodes[current_pos[1] + 1][current_pos[0]] # Above
            self.__test_node(node, open_set, current_node, 10)

        if (current_pos[1] - 1 >= 0):
            node = grid_nodes[current_pos[1] - 1][current_pos[0]] # Below
            self.__test_node(node, open_set, current_node, 10)


        if (current_pos[0] + 1 <= self.__x_lim and current_pos[1] + 1 < self.__y_lim):
            node = grid_nodes[current_pos[1] + 1][current_pos[0] + 1] # Top Right
            self.__test_node(node, open_set, current_node, 14)

        if (current_pos[0] - 1 >= 0 and current_pos[1] + 1 < self.__y_lim):
            node = grid_nodes[current_pos[1] + 1][current_pos[0] - 1] # Top Left
            self.__test_node(node, open_set, current_node, 14)

        if (current_pos[0] + 1 <= self.__x_lim and current_pos[1] - 1 > 0):
            node = grid_nodes[current_pos[1] - 1][current_pos[0] + 1] # Bottom Right
            self.__test_node(node, open_set, current_node, 14)

        if (current_pos[0] - 1 >= 0 and current_pos[1] - 1 > 0):
            node = grid_nodes[current_pos[1] - 1][current_pos[0] - 1] # Bottom Left
            self.__test_node(node, open_set, current_node, 14)


    def __test_node(self, node, open_set, current_node, cost):
        if (node.isclosed() == False and node.getObstacle() == False):
            if (node not in open_set):
                open_set.append(node) # Right
                node.setG(current_node.getG() + cost)
                node.calcF()
                node.setParent(current_node)
            elif (current_node.getG() + cost < node.getG()):
                node.setParent(current_node)


    def __find_smallest(self, open_set):
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




    def set_start_pos(self, x, y): # Method to set the x and y start position of the robot
        self.start_pos[0] = x
        self.start_pos[1] = y


    def set_start_rot(self, rotation): # Method to set the start rotation of the robot
        self.start_rot = rotation

    def get_start_pos(self): # Method to return the start position
        return self.start_pos

    def get_start_rot(self): # Method to return the start rotation
        return self.start_rot


    def print_grid(self): # Print out the grid, with 0 meaning no obstacle and 1 an obstacle
        for i in range (self.__y_lim,-1,-1):
            print(self.grid_map[i])


    def __del__(self):
        self.reset_position() # Reset robot to start position
        self.set_led_green(0) # Set green led off
        self.set_led_red(0) # Set red led off
        self.cleanup() # Cleanup gpio pins





























