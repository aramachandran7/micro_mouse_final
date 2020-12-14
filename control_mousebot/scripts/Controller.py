#!/usr/bin/env python3

""" objective - control the robot at a very very high level"""
from Graph import Graph2
from MoveComputer import MoveComputer2
from DriveStep import DriveStep
import rospy
import time



def run():
    # setting constants
    speed = 0.3
    unit_length = 0.193
    # pos = (0,0)
    pos = (0,0)
    target = (7.5, 7.5)
    len_nodes_visited = 5
    nodes_visited = [0]*len_nodes_visited
    pointer = 0

    # creating objects
    MoveComputer = MoveComputer2()
    graph = Graph2()
    driver = DriveStep(pos, unit_length=unit_length)
    # time.sleep(3) # hold up
    walls = driver.return_walls(first=True) # compute first walls before movement
    print('walls returned first: ', walls)

    #print(graph.graph)
    # blocking code while loop for mousebot reach center
    while pos != target and not rospy.is_shutdown():
        print('movement!=======================================================================')
        graph.update_graph(pos, walls)
        next_pos = MoveComputer.compute_next_move(graph, pos)
        print("Moving to:   ", next_pos)
        walls = driver.drive(next_pos, speed) # updates walls, position
        pos = next_pos

        nodes_visited[pointer] = pos

        counter = 0
        for visited in nodes_visited:
            if visited == nodes_visited[pointer]:
                counter += 1
        if counter >=3:
            print("you're in a back and forth loop, altering MoveComputer2 code")
            MoveComputer.coef = 3.0
        # else:
        #     MoveComputer.coef = 1.0

        pointer = (pointer+1)%len_nodes_visited
        print(' ')
        time.sleep(.3)
    # code for mousebot to reverse track back to starting point

    # graph.center = (0,0)
    # ...

    # code for mousebot speedrun.

    # optimized_path = path_planner.generate_optimal(graph)
    # driver.speed_run(optimized_path)

if __name__ == '__main__':
    run()
