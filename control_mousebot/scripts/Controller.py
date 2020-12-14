#!/usr/bin/env python3

""" objective - control the robot at a very very high level"""
from Graph import Graph2
from MoveComputer import MoveComputer2
from DriveStep import DriveStep
from PathPlanner import PathPlanner
import rospy
import time
import numpy as np



def run():
    # setting constants
    speed = 0.3
    unit_length = 0.193
    # pos = (0,0)
    pos = (0,0)
    target_list = [(7, 7), (7, 8), (8, 7), (8, 8)]
    len_nodes_visited = 5
    nodes_visited = [0]*len_nodes_visited
    pointer = 0

    # creating objects
    MoveComputer = MoveComputer2()
    graph = Graph2()
    driver = DriveStep(pos, unit_length=unit_length)
    planner = PathPlanner()

    # time.sleep(3) # hold up
    walls = driver.return_walls(first=True) # compute first walls before movement
    print('walls returned first: ', walls)

    # test pathplanning code
    target = (8,7)
    previous_graph = np.load('mazes/graph.npy',allow_pickle='TRUE').item()
    path_to_center = planner.a_star(previous_graph, start=pos, target=target)
    for position in path_to_center:
        print('movement!=======================================================================')
        print("Moving to:   ", position)
        walls = driver.drive(position, speed) # updates walls, position
        pos = position
        print(' ')
        time.sleep(.3)

    print("At Center.")






    # blocking code while loop for mousebot reach center
    # while (pos not in target_list) and not rospy.is_shutdown():
    #     print('movement!=======================================================================')
    #     graph.update_graph(pos, walls)
    #     next_pos = MoveComputer.compute_next_move(graph, pos)
    #     print("Moving to:   ", next_pos)
    #     walls = driver.drive(next_pos, speed) # updates walls, position
    #     pos = next_pos
    #
    #     nodes_visited[pointer] = pos
    #
    #     counter = 0
    #     for visited in nodes_visited:
    #         if visited == nodes_visited[pointer]:
    #             counter += 1
    #     if counter >=3:
    #         print("you're in a back and forth loop, altering MoveComputer2 code")
    #     # else:
    #     #     MoveComputer.coef = 1.0
    #
    #     pointer = (pointer+1)%len_nodes_visited
    #     print(' ')
    #     time.sleep(.3)
    #
    # print("reached center")
    # # save dictionary
    # np.save('mazes/graph.npy', graph.graph)
    #
    # # code for mousebot to reverse track back to starting point
    # target = (0,0)
    # path_to_home = planner.a_star(graph.graph, start=pos, target=target)
    #
    # for position in path_to_home:
    #     print('movement!=======================================================================')
    #     print("Moving to:   ", position)
    #     walls = driver.drive(position, speed) # updates walls, position
    #     pos = position
    #     print(' ')
    #     time.sleep(.3)
    #
    # print("Back home.")


if __name__ == '__main__':
    run()
