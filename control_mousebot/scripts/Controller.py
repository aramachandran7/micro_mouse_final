#!/usr/bin/env python3

""" objective - control the robot at a very very high level"""
from Graph import Graph2
from MoveComputer import MoveComputer2
from DriveStep import DriveStep
from PathPlanner import PathPlanner
import rospy
import time
import numpy as np

class Controller(object):
    def __init__(self):

        # params for driving
        self.speed = 0.3
        unit_length = 0.188

        # positions
        self.pos = (0,0)
        self.target_list = [(7, 7), (7, 8), (8, 7), (8, 8)]

        # circular buffer for nodes visited
        self.len_nodes_visited = 5
        self.nodes_visited = [0]*self.len_nodes_visited
        self.pointer = 0

        # initializing objects
        self.MoveComputer = MoveComputer2()
        self.graph = Graph2()
        self.driver = DriveStep(self.pos, unit_length=unit_length)
        self.planner = PathPlanner()

        self.previous_graph = None # dictionary, not graph object.

        self.save = False # for saving the graph
        self.step = True # steps vs continuous drive


    def run(self):
        """ blocking code while loop for mousebot reach center """
        walls = self.driver.return_walls(first=True) # compute first walls before movement
        print('walls returned first: ', walls)
        pos = self.pos
        while (pos not in self.target_list) and not rospy.is_shutdown():
            print('movement!=======================================================================')
            self.graph.update_graph(pos, walls)
            next_pos = self.MoveComputer.compute_next_move(self.graph, pos)
            print("Moving to:   ", next_pos)
            walls = self.driver.drive(next_pos, self.speed) # updates walls, position
            pos = next_pos

            self.nodes_visited[self.pointer] = pos

            counter = 0
            for visited in self.nodes_visited:
                if visited == self.nodes_visited[self.pointer]:
                    counter += 1
            if counter >=3:
                print("you're in a back and forth loop, altering MoveComputer2 code")
            # else:
            #     MoveComputer.coef = 1.0

            self.pointer = (self.pointer+1)%self.len_nodes_visited
            print(' ')
            if self.step:
                time.sleep(.3)

        self.graph.update_graph(pos, walls) # final graph update with destination. 

        print("reached center")
        # save dictionary
        if self.save:
            time.sleep(5)

            print("should be complete graph, saving ... ", self.graph.graph)
            np.save('mazes/graph.npy', self.graph.graph)

    def test_pathplanning(self):
        """test pathplanning code with imported maze graph"""

        target = (8,8)
        pos = self.pos
        self.previous_graph = np.load('mazes/graph.npy',allow_pickle='TRUE').item()

        print("prev graph: ", self.previous_graph)

        path_to_center = self.planner.a_star(self.previous_graph, start=pos, target=target)
        while not rospy.is_shutdown():
            for position in path_to_center:
                print('movement!=======================================================================')
                print("Moving to:   ", position)
                walls = self.driver.drive(position, self.speed) # updates walls, position
                pos = position
                print(' ')
                if self.step:
                    time.sleep(.3)
            break

        print("At Center.")


    def get_home(self):
        # code for mousebot to reverse track back to starting point
        target = (0,0)
        path_to_home = self.planner.a_star(self.graph.graph, start=self.pos, target=target)
        pos = self.pos
        while not rospy.is_shutdown():
            for position in path_to_home:
                print('movement!=======================================================================')
                print("Moving to:   ", position)
                walls = self.driver.drive(position, speed) # updates walls, position
                pos = position
                print(' ')
                if self.step:
                    time.sleep(.3)
            break

        print("Back home.")

    def speedrun(self):
        pos = self.pos
        target = (8,8)
        path_to_center = self.planner.a_star(self.graph.graph, start=pos, target=target)
        while not rospy.is_shutdown():
            for position in path_to_center:
                print('movement!=======================================================================')
                print("Moving to:   ", position)
                walls = self.driver.drive(position, self.speed) # updates walls, position
                pos = position
                print(' ')
                if self.step:
                    time.sleep(.3)
            break

        print("At Center.")



if __name__ == '__main__':
    control = Controller()
    control.test_pathplanning()
    #control.run()
    #control.get_home()
    #control.speedrun()
    # control.run()
