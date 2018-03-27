import numpy as np
from global_variables import *

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        # location of the robot
        self.location = [0, 0]
        # the robot's direction
        self.heading = 'up'
        # dimension of the maze
        self.maze_dim = maze_dim
        # area of the maze
        self.maze_area = float(maze_dim ** 2)
        # maze grid for wall locations
        self.maze_grid = np.zeros((maze_dim, maze_dim), dtype=int)
        # checks if goal has been discovered during exploratory trial
        self.discovered_goal = False
        # grid for path locations
        self.grid_path = np.zeros((maze_dim, maze_dim), dtype=int)
        # score from goal and back to the start
        self.path_value = [[99 for row in range(maze_dim)] for col in range (maze_dim)]
        # maps grid to optimal roure
        self.policy_grid = [[' ' for row in range(maze_dim)] for col in range (maze_dim)]
        # goal area
        self.goal_area = [maze_dim / 2 - 1, maze_dim / 2] or [maze_dim / 2, maze_dim / 2 - 1]
        # heuristic grid
        self.heuristic = [[min(abs(row - maze_dim / 2 + 1), abs(row - maze_dim / 2)) + min(abs(col - maze_dim / 2 + 1), abs(col - maze_dim / 2)) for row in range(maze_dim)] for col in range(maze_dim)]
        # number of steps taken in each trial
        self.step_count = 0
        # initial value of downwards direction
        self.backwards = 0
        # exploratory or optimal trial
        self.run_trial = 0

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        # checks if robot has commenced any trial
        if self.run_trial == 0:
            rotation, movement = self.exploratory_trial(sensors)

        # runs optimal trial after exploratory trial
        elif self.run_trial == 1:
            rotation, movement = self.optimization_trial(sensors)

        return rotation, movement

    def control_next_move(self, x1, y1, sensors):
        # if the robot enters into a dead-end, it reverses
        if sensors == [0, 0, 0] and self.heading == 'u' and self.maze_grid[x1][y1 - 1] == 5:
            movement = -2
            rotation = 0
            print 'Robot hit a dead-end... reversing...'

        elif sensors == [0, 0, 0] and self.heading == 'r' and self.maze_grid[x1 - 1][y1] == 10:
            movement = -2
            rotation = 0
            print 'Robot hit a dead-end... reversing...'

        elif sensors == [0, 0, 0] and self.heading == 'd' and self.maze_grid[x1][y1 + 1] == 5:
            movement = -2
            rotation = 0
            print 'Robot hit a dead-end... reversing...'

        elif sensors == [0, 0, 0] and self.heading == 'l' and self.maze_grid[x1 + 1][y1] == 10:
            movement = -2
            rotation = 0
            print 'Robot hit a dead-end... reversing...'

        elif sensors == [0, 0, 0]:
            movement = -1
            rotation = 0
            print 'Robot hit a dead-end... reversing...'

        # this implements A*, the robot moves to an open space
        else:
            possible_moves = []
            rotation_angles = [-90, 0, 90]

            # sensor readings: 1 is open and 0 for closed wall
            for sensor in range(len(sensors)):
                if sensors[sensor] > 0:
                    sensors[sensor] = 1

                    # robot moves to the next open space
                    x2 = x1 + dir_move[dir_sensors[self.heading][sensor]][0]
                    y2 = y1 + dir_move[dir_sensors[self.heading][sensor]][1]

                    # makes sure next open space is in the maze
                    if x2 >= 0 and x2 < self.maze_dim and y2 >= 0 and y2 < self.maze_dim:
                        # calculates the number of times this cell has been visited
                        visited_cell = self.grid_path[x2][y2]
                        # calculates heuristics: a number that indicates the distance to the goal area
                        heuristics_value = self.heuristic[x2][y2]
                        # makes a list of possible moves for the robot
                        possible_moves.append([visited_cell, heuristics_value, x2, y2, sensor])

            # sort the list of possible_moves
            possible_moves.sort()
            # remove the smallest value, which is probably closest to the center or that is yet to be explored
            smallest_value = possible_moves.pop(0)
            # break the values into separate variables
            v_value, h_value, x_value, y_value, sensor_val = smallest_value
            # rotate to opening and move 1 step
            rotation, movement = rotation_angles[sensor_val], 1

        return rotation, movement

    # this function updates the robot's direction and location
    def update_location(self, rotation, movement):
        # converts rotation angles to index key of [0,1,2]
        if rotation == -90 or rotation == 270:
            rotation = 0

        elif rotation == 0 or rotation == 360:
            rotation = 1

        elif rotation == 90 or rotation == -270:
            rotation = 2

        # computes new direction based on robot's rotation
        self.heading  = dir_sensors[self.heading][rotation]

        # updates location
        self.location[0] += dir_move[self.heading][0] * movement
        self.location[1] += dir_move[self.heading][1] * movement

    # this function creates a number that represents description of walls surrounding the robot
    def wall_locations(self, sensors, backwards):
        '''
        sensor readings are left, front, right represented as [left, front, right]
        1 represents open and 0 represents closed wall
        '''
        for sensor in range(len(sensors)):
            if sensors[sensor] > 0:
                sensors[sensor] = 1

        # 1 = North, 2 = East, 4 = South, 8 = West
        if self.heading == 'u' or self.heading == 'up':
            value = (sensors[0] * 8) + (backwards * 4) + (sensors[2] * 2) + sensors[1]

        elif self.heading == 'r' or self.heading == 'right':

            value = (backwards * 8) + (sensors[2] * 4) + (sensors[1] * 2) + sensors[0]
        elif self.heading == 'd' or self.heading == 'down':
            value = (sensors[2] * 8) + (sensors[1] * 4) + (sensors[0] * 2) + backwards

        elif self.heading == 'l' or self.heading == 'left':
            value = (sensors[1] * 8) + (sensors[0] * 4) + (backwards * 2) + sensors[2]

        return value

    # this is the first trial for the robot to have a knowledge of the maze
    def exploratory_trial(self, sensors):
        # counts the number of steps taken
        print 'Exploration Trial; Number of steps: ', self.step_count, sensors, self.location
        self.step_count +=1

        # retrieves robot's current location
        x1 = self.location[0]
        y1 = self.location[1]

        self.grid_path[x1][y1] += 1

        # calculates the percentage of the maze the robot has discovered
        discovered = 0
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                if self.grid_path[x][y] > 0:
                    discovered += 1

        discovered_area = (discovered / self.maze_area) * 100
        print 'Robot has discovered %.2f%% of the maze.\n' % discovered_area

        # draws map of the maze from the sensor readings
        map_of_maze = self.wall_locations(sensors, self.backwards) 
        self.maze_grid[x1][y1] = map_of_maze

        # determines the robot's next move
        rotation, movement = self.control_next_move(x1, y1, sensors)

        # update the value of downwards direction
        if movement == 0:
            self.backwards = 0

        elif movement == 1:
            self.backwards = 1

        # if robot hits a dead-end
        elif movement < 0:
            for pos in range(2):
                # if the robot is facing left: 9, 12 and 13 have a wall on their right side
                if self.heading == 'l' or self.heading == 'left':
                    if self.maze_grid[x1 + pos][y1] == 9 or self.maze_grid[x1 + pos][y1] == 12 or self.maze_grid[x1 + pos][y1] == 13:
                        self.backwards = 0
                    else:
                        self.backwards = 1

                # if the robot is facing right: 3, 6 and 7 have a wall on their left side
                elif self.heading == 'r' or self.heading == 'right':
                    if self.maze_grid[x1 - pos][y1] == 3 or self.maze_grid[x1 - pos][y1] == 6 or self.maze_grid[x1 - pos][y1] == 7:
                        self.backwards = 0
                    else:
                        self.backwards = 1

                # if the robot is facing up: 3, 9 and 11 have a wall on their bottom side
                elif self.heading == 'u' or self.heading == 'up':
                    if self.maze_grid[x1][y1 - pos] == 3 or self.maze_grid[x1][y1 - pos] == 9 or self.maze_grid[x1][y1 - pos] == 11:
                        self.backwards = 0
                    else:
                        self.backwards = 1
                
                # if the robot is facing down: 6, 12 and 14 have a wall on their top side
                elif self.heading == 'd' or self.heading == 'down':
                    if self.maze_grid[x1][y1 + pos] == 6 or self.maze_grid[x1][y1 + pos] == 12 or self.maze_grid[x1][y1 + pos] == 14:
                        self.backwards = 0
                    else:
                        self.backwards = 1

        # updates the robot's direction and location
        self.update_location(rotation, movement)

        # this gets the robot's new location
        x2 = self.location[0]
        y2 = self.location[1]

        # checks if robot's new location is the goal area
        if x2 in self.goal_area and y2 in self.goal_area:
            if self.grid_path[x2][y2] == 0:
                print '------------------------------------'
                print "***Yay! Robot found the goal area after {} steps. ***\n".format(self.step_count)
                print 'Robot is still exploring the maze...'
                self.discovered_goal = True

        # checks if goal area has been discovered and robot has visited about 70% of the maze
        elif self.discovered_goal and discovered_area >= 70:
            print '------------------------------------'
            print 'Robot has stopped exploring the maze...'
            # save goal area result in a variable
            goal = self.goal_area
            # computes result
            self.compute_value(goal)
            print '\n---  Maze Grid  ---\n', self.maze_grid
            print '\n---  Grid Path  ---\n', self.grid_path
            print '\n---  Path Value  ---\n', self.path_value
            print '\n---  Policy Grid  ---\n', self.policy_grid

            # reset settings and start optimization trial
            rotation = 'Reset'
            movement = 'Reset'
            self.reset_values()

        return rotation, movement

    # reset settings and start optimization trial
    def reset_values(self):
        # sets run_trial to 1, means that exploratory trial has been done
        # and it initiates the optimization trial
        self.run_trial = 1
        self.location = [0, 0]
        self.step_count = 0
        self.heading = 'up'
        self.discovered_goal = False

    # this is the second trial, the robot tries to follow the optimal path
    def optimization_trial(self, sensors):
        # counts the number of steps taken
        print 'Optimization Trial; Number of steps: ', self.step_count, sensors, self.location
        self.step_count += 1

        movement = 1

        # retrieves robot's current location
        x1 = self.location[0]
        y1 = self.location[1]

        # rotate to the optimal path
        heading_angle = dir_motion[self.heading]
        optimal_heading_angle = dir_motion[self.policy_grid[x1][y1]]
        rotation = optimal_heading_angle - heading_angle

        # checks if rotation is 270 or -270 degrees and return the appropriate value
        if rotation == 270:
            rotation = -90

        elif rotation == -270:
            rotation = 90

        # change the angles to index of [0, 1, 2]
        rotation_index = rotation / 90 + 1
        # change direction
        direction = dir_sensors[self.heading][rotation_index]

        # move up to three(3) consecutive steps
        while movement < 3:
            location = self.policy_grid[x1][y1]
            x1 += dir_move[direction][0]
            y1 += dir_move[direction][1] 

            if self.policy_grid[x1][y1] == location:
                movement += 1
            else:
                break

        # update robot's direction and location
        self.update_location(rotation, movement)

        # retrieve robot's new location
        x2 = self.location[0]
        y2 = self.location[1]
        print "Robot's Location: ", self.location

        return rotation, movement

    def compute_value(self, goal):
        change = True

        while change:
            change = False

            for x in range(self.maze_dim):
                for y in range(self.maze_dim):
                    if goal[0] == x and goal[1] == y:
                         # this prevents running into an endless loop
                        if self.path_value[x][y] > 0:
                            # assigns 0 to the goal position in the path value grid
                            self.path_value[x][y] = 0
                            # assigns * to the goal position in the policy grid
                            self.policy_grid[x][y] = '*'
                            print '------------------------------------'
                            print "Goal location: {}\n".format(goal)
                            change = True

                    else:
                        # converts the wall value into 4-bit
                        wall = self.maze_grid[x][y]
                        binary = int(bin(wall)[2:])
                        four_bit = '%0*d' % (4, binary)

                        # starts at goal and increase by 1 in open spaces
                        for direction in range(len(delta)):
                            if (four_bit[0] == '1' and direction == 0) or (four_bit[1] == '1' and direction == 1) or (four_bit[2] == '1' and direction == 2) or (four_bit[3] == '1' and direction == 3):
                                x2 = x + delta[direction][0]
                                y2 = y + delta[direction][1]

                                # makes sure open space is in the maze
                                if x2 >= 0 and x2 < self.maze_dim and y2 >= 0 and y2 < self.maze_dim:
                                    # add 1 to path value; each step costs 1
                                    v2 = self.path_value[x2][y2] + 1

                                    if v2 < self.path_value[x][y]:
                                        change = True
                                        # update path_value
                                        self.path_value[x][y] = v2
                                        # add movement symbol to policy_grid
                                        self.policy_grid[x][y] = delta_symbol[direction]

# some parts of this code is from Udacity's AI for robotics course
# https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373
# http://swarm.cs.pub.ro/~anpetre/dynamic_prog.pdf
