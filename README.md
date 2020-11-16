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
