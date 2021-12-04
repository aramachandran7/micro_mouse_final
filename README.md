# micro_mouse_final
Comprobo final project featuring SLAM, maze traversal + path planning, robot dynamics with Gazebo &amp; ROS

## ALL Project docs @ [GH-Pages Branch](https://github.com/aramachandran7/micro_mouse_final/blob/gh-pages/index.md).

### Checkout the videos below - speedrun & full challenge completion!

[![Speedrun](https://img.youtube.com/vi/Khu0GxKtttg/0.jpg)](https://www.youtube.com/watch?v=Khu0GxKtttg )

[![Full Challenge Completion](https://img.youtube.com/vi/e3KHShxfUm0/0.jpg)](https://www.youtube.com/watch?v=e3KHShxfUm0 )


### Future improvements
- if no new information (walls) & path contains no new informaiton (new_walls ~= prev_walls) unknown, planning ahead 
- Rework discrete stepping and turning at each unit square. This is a significant factor in our final run time and removing this pause between movements, such as we did in the speed_run, would time-optimize the longest part of our run.
- The speed run could be further improved by combining the turning and driving function. Turning while continuing to drive forward would remove the need to stop entirely. This would both decrease run time and reduce error from jerking.
- Looking at the path planning, a future improvement would be to change how we define the fastest path. Rather than necassarily optimizing for the shortest overall path from point A to point B, we could alter our A star algorithm to find paths with more long straightaways, that would be faster in the speed run.
- Spending more time to fine tune our MoveComputer’s confidence computations to have logic that works on any maze not just this one is a good idea 
- Outside of our algorithm, we would love to have the time to test our mousebot in a few different mazes. Our code likely has many parts that work for this maze alone. Testing other mazes is a necessity for robust move computation and pathplanning.
