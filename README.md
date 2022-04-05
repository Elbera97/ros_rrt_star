# OMPL_RRTstar_ROS
This is an OMPL RRT* algorithm ROS node implementation

# Requirements
- 18.04 Ubuntu 
- ROS Melodic
- OMPL library
- Eigen3 library

# Installation
1. Open a terminal
2. In `src` folder of a ROS workspace run `git clone https://github.com/Elbera97/ros_rrt_star.git`
3. Go back to workspace folder `cd ..` 
4. Run `catkin_make`

# Parameters
In `launch/rrt_star.launch` file there are some parameters you can play with:
- "length" : length of the map in meters (y direction)
- "width" : width of the map in meters (x direction)
- "rrtstar_cost" : if rrtstar_cost = 0 RRT* seeks for optimal solution until termination_time
- "termination_time" : if rrtstar_cost > 0 RRT* may stop before termination_time if a solution satisfying rrtstar_cost is found

# Launch
1. Run `roslaunch ros_rrt_star rrt_star.launch`
2. Open new tab and run: 
`rosservice call /rrt_star_planning "{start_x: -50.0, start_y: -50.0, end_x: 50.0, end_y: 50.0, radius: 5.0, obstacle_num: 40}"`

# Demo 
![This is an image](https://github.com/Elbera97/ros_rrt_star/blob/main/demo.png)
	
