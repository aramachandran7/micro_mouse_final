
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
    walls,pos = driver.drive('F', speed)
    graph.update_graph(pos, walls)
    print(graph.graph)
    # blocking code while loop for mousebot reach center
    # while pos != center:
    #
    #     next_pos = MoveComputer.compute_next_move(graph, pos)
    #     walls,pos = driver.drive(next_pos, speed) # updates walls, position
    #     pos = next_pos
    #     graph.update_graph(pos, walls)

    # code for mousebot to reverse track back to starting point

    # graph.center = (0,0)
    # ...

    # code for mousebot speedrun.

    # optimized_path = path_planner.generate_optimal(graph)
    # driver.speed_run(optimized_path)

if __name__ == __main__:
    run()
