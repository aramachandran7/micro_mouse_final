# micro_mouse_final
Comprobo final project featuring SLAM, maze traversal + path planning, robot dynamics with Gazebo &amp; ROS


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
