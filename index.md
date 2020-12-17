# Welcome to our comprobo final!

## What we did!
The goal of this project was to design and program a robot that could complete the Micromouse competition. Our custom differential drive bot autonomously navigates and solves a 16x16 maze, finding its way to the center on the first run. It then returns to its starting point by generating a path from its understanding of the maze. And lastly, it pathplans an optimized route for a final thrilling speedrun to the center.

The project involved graph theory & recursive algorithm design, high speed robot motion control, discrete SLAM with LIDAR & odometry (so not really slam, lol), and pathplanning! `Great Success!`

[![Micromouse video](https://img.youtube.com/vi/A4hzCcFikm0/0.jpg)](https://www.youtube.com/watch?v=A4hzCcFikm0)
*video, click to watch*

## A sneak peek at some interesting challenges we faced
- The maze has loops! What happens when your bot is looking for the center but after exploring ends up back in the same place it was 20 steps ago?
- Custom robot models in ROS grant you immense control over their sensor output, but you have to pay close attention.
- Gazebo simulation issues (lidar clipping through walls, differential drive)

## Project Motivations
The micromouse challenge was a great project in that it was a solid combination of 3D modeling / design, robot localization + graph theory, pathfinding, and robot motor control & dynamics. It also provided us with a nice, clear end goal from the start. 

## The high level robot controller, explained
The high level control is the easiest way to understand our system from an abstracted point of view.
The `Controller` class handles all top level arbitration, and calls all of our lower level API's to do any computation with routing, pathplanning, or interface with ROS via pubsub for robot control.  

![block_diagram](block_diagram.png)
*System block diagram*

As seen in the system diagram above, there are 2 main modes of operation for our controller - **Maze Solving** and **Speed_run**.

### Maze Solving
Here the robot has *no* previous understanding of the maze, other than knowledge of the target / center's location.  
It uses the `DriveStep` API to interface with ROS, which abstracts away reading and interpreting odometery and LIDAR scan data, and discrete robot control in unit steps within the maze. It passes the data returned from `DriveStep` (namely the presence or absence of the N E S W wall directions at the bot's current position) into the graph. The Graph is used to store the bot's understanding of the maze.

The `MoveComputer` takes the current graph and computes the next best move for the bot - it can robustly handle dead-ends, loops, and not knowing anything about the maze at all! Finally, the `MoveComputer` passes the next move to the `Controller`, and the entire process is repeated until the center is reached.

### Speed_run
Here, the robot already has maze data stored in the graph (this state of controller will only ever be entered after `Maze Solving` is completed). `Controller` uses `PathPlanner` to optimize a route to either the center or back home. Optimized routes are stored as a list of 'end nodes' for the bot to reach. The `Controller` then walks through all the optimized route and calls the `speed_run` function within DriveStep for every step.   

## Lower level API, in detail

### DriveStep (linreg, sigmoidal control, compute_keypoints)
We needed our maze navigation class to be as robust and independent as possible so it could easily work in parallel with a variety of higher level algorithms. The goal of DriveStep was to handle all of the robot's *local* positioning and control, including keeping account of the bot's orientation, ensuring its position was centered within every *unit* in the maze, and reading + interpreting environment sensor data.

Using 100Hz LIDAR scan data, we would check for walls nearby and calculate our bot's skew angle (either to the front, left, right or back wall). The `skew` variable held our angular skew, linear skew, and distance to the closest wall, and was computed using linear regression. Upon finding the closest wall, a matrix of polar coordinates was created from the LIDAR scan points. After re-centering the angles to spread around zero degrees, the bot creates a regression line who's slope defined its angular skew against that wall. Centering the angles before the linear regression allowed us to use angle skew as an objective measure of our error regardless of the bot's global oreintation (N E S W).

Once we had the skew for a given scan, we fed it to either our turning or driving method to move the robot. To achieve the optimal speed for both turning and course correction, we employed sigmoidal functions to connect our current orientation with angular velocity. Reducing jerking improved our turning accuracy, especially for 180 degree turns.

To ensure accurate distance traveled calculations, we implemented an additional method to detect keypoints in the maze. These 'keypoints' were edges of the walls signifying either the entrance to a path or a corner. By checking the 45 and 315 degree LIDAR scans, were able to **recognize the end of a unit square** within the maze, without needing a wall directly in front of the bot. 

### MoveComputer & Graph
One of our major design decisions was to store the robot's current understanding of the maze as a graph. This data structure needed to be ... 
- Fast to add information to as the robot moves through the maze
- Fast to access information from
- Have a usable API for traversals and computations
- Has a high level of visibility or is easy to visualize for debugging

Read more about this design decision in **blog post 1**. Every position the robot can be in within the graph is a node, and if the robot can move between two positions in the maze (there isn't a wall between those two discrete positions), the two nodes are connected. As the robot moves through the maze, it adds positions its visited as nodes to the graph.

The `MoveComputer` processes the graph and the robot's current position, and computes a next best move. It evaluates each direction the robot can go (or node connected to the current one) by computing a confidence. The confidence for a direction is based off
- the distance to the nearest *unknown*, or a connected node that hasn't yet been visited
- the distance from that unknown to the center.

We perform a depth-limited recursive depth-first search for every direction in the graph to find the nearest unknown in that direction. We depth limit our DFS to prevent getting caught in loops (which is commonplace in this maze)!

### PathPlanner (our custom implementation of A*)
The `PathPlanner` conssits of 2 components
- The `A*` algorithm implemented in python to interface with our graph and compute an optimized route between two unique nodes.
- A route optimization function `consolidate_path` that turns a list of nodes into a list of 'end nodes', which helps our `speedrun` function understand how long its next straightway stretch.     

Our implementation of A Star uses the F, G, and H heuristics, walking through and adding to a cost-sorted priority queue until it empties the queue or finds the target node. We use Euclidean distance as our heuristic and of course, children are limited by connected nodes (portions of the maze the bot can traverse) and nodes actually in the graph (portions of the maze the bot *did* traverse during Maze Solving).

### A special shout-out to `speed_run`!
The `speed_run` functionality of our `DriveStep` API takes the optimized route generated from the `PathPlanner`, and  uses a custom proportional control to set bot speed as it drives down long straightways in the route. 

Unlike in the pro micromouse challenge, our turns are still seperate from driving forwards (and funciton identically to our turns during Maze Solving), we no longer drive 'step by step' during straight portions of the robot path. The bot relies heaviliy on odometry to drive down long stretches and can consistently pick up to speeds up to 3x higher than it would when first solving the maze.   

### A special shout-out to robot design!
The mousebot model for this project was uniquely designed for the gazebo maze. We designed a very simple two wheeled differential drive robot in SolidWorks to the size specifications given in the mousebot competition rules. Custom designing the robot meant that we could simplify our model to two wheels and a LIDAR. We imported the model using the URDF exporter, and were able to further critique the design for troubleshooting directly in the URDF file.

(this section could use a revisit)

## Challenges we faced

### Loops are hard
The maze has loops, which The robot got into 

## Reflection & future improvements
The first aspect of our mapping run that could use improvement is the discrete stepping and turning at each unit square. This is a significant factor in our final run time and removing this pause between movements, such as we did in the speed_run, would time-optimize the longest part of our run. 


The speed run could be further improved by combining the turning and driving function. Turning while continuing to drive forward would remove the need to stop entirely. This would both decrease run time and reduce error from jerking.


Looking at the path planning, a future improvement would be to change how we define the fastest path. Rather than necassarily optimizing for the shortest overall path from point A to point B, we could alter our A star algorithm to find paths with more long straightways, that would be faster in the speed run. 

Spending more time to fine tune our MoveComputer's confidence 

Outside of our algorithm, we would love to have the time to test our mousebot in a few different mazes. Our code likely has many parts that work for this maze alone. Testing other mazes is a necessity for robust move computation and pathplanning. 