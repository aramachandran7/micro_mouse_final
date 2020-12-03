# micro_mouse_final
Comprobo final project featuring SLAM, maze traversal + path planning, robot dynamics with Gazebo &amp; ROS
##todos
de-acceleration


## Week by Week Goals

Week 1: Working robot model in gazebo, working base maze map. Get the robot to drive straight for a distance constrained by the maze walls, and modularize the code. ROS setup, 3D modeling, and controls.

Week 2: Make note of and have the robot recognize dead-ends, intersections, and typical maze characteristics. Discuss with Paul on simultaneous localization  within the map context, (moving towards the maze destination), and map/context building. Be able to represent the robot’s position within a discrete matrix. Depending on our implementation, be able to represent map?

Week 3: Completing mapping & SLAM. Beginning work on path optimization Robot should be able to reach the center of the map with SLAM, and should be able to get back using a different route. .  

Week 4: Finishing up path planning, optimizing high speed performance for 2nd run and onwards.  

Week 5 (half week): Finishing up path planning, optimizing high speed performance for 2nd run and onwards, and also documentation / writeup / slides. Blog posts  


#### super high level code structure -

```
map_understanding = [0]
map_understanding = pointAtoB_andmap(map_understanding)
map_understanding = pointAtoB_andmap(center, map_understanding)
optimized_path = pathplan(map_understanding)

speedrun(optimized_path)

```

1) mapping while moving, trying to get to center
the drive function is given a large global graph (65**2) in size.
The robot computes based on lidar the edges between the 4 immediate nodes.
It computes the next move (N or E or W or S) based on its understanding of the global graph
Then it moves 1 unit in that direction, repeat.  
```
scanner.scan_graph(graph)
computer.compute_next_move()
driver.drive()
```

1.5)
path plannign wtf


2) traversing a path that has already been figured out
Given an optimized_path = [N,N,N,E,E,S,W]
It follows that path

Challenges
using /odom to stay centered / in discrete.

driving requiremnet - drive 1 unit N E S W, return encoder data?


```
# (mapunits, degrees)
# maintain a constant heading,



driver = DriveStep(speed, units=.18)
[n n e ]
driver.drive('N', 18)
coputaiton()
driver.drive('E',25)

```
