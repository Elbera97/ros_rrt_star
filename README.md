# OMPL_RRTstar_ROS
This is an OMPL RRT* algorithm ROS node implementation

# Requirements
-Ubuntu 
-ROS
-OMPL

# How to run
1. Open a terminal
2. In `src` folder of a ROS workspace `git clone https://github.com/Elbera97/ros_rrt_star.git`
3. `cd ..` 
4. `catkin_make`
5. `roslaunch ros_rrt_star rrt_star.launch`
6. open new tab and run: 
`rosservice call /rrt_star_planning "{start_x: -50.0, start_y: -50.0, end_x: 50.0, end_y: 50.0, radius: 5.0, obstacle_num: 40}"`

# Parameters

# Demo 

