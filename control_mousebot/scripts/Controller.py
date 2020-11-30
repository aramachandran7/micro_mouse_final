#!/usr/bin/env python3

""" objective - control the robot at a very very high level"""
from Graph import Graph2
from MoveComputer import MoveComputer2
from DriveStep import DriveStep



def run():
    # setting constants
    speed = 0.2
    pos = (0,0)
    walls = None

    # creating objects
    MoveComputer = MoveComputer2()
    graph = Graph2()
    driver = DriveStep()

    # make first move, grab first data
    #walls = driver.drive((0,1), speed)
    #pos = (0,1)
    #graph.update_graph(pos, walls)
    # walls = driver.drive((0,2), speed)
    # pos = (0,2)
    # graph.update_graph(pos, walls)

    target = (7.5, 7.5)
    print(graph.graph)
    # blocking code while loop for mousebot reach center
    while pos != target:
        print('movement!---------------------')
        graph.update_graph(pos, walls)
        next_pos = MoveComputer.compute_next_move(graph, pos)
        print("move comptuer returned, ", next_pos)
        walls = driver.drive(next_pos, speed) # updates walls, position
        pos = next_pos

    # code for mousebot to reverse track back to starting point

    # graph.center = (0,0)
    # ...

    # code for mousebot speedrun.

    # optimized_path = path_planner.generate_optimal(graph)
    # driver.speed_run(optimized_path)

if __name__ == '__main__':
    run()
