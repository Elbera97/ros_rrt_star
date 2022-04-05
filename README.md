# OMPL_RRTstar_ROS
This is an OMPL RRT* algorithm ROS node implementation

# How to run
1. git clone in ws/src
2. cd .. 
3. catkin_make
4. roslaunch ros_rrt_star rrt_star.launch
5. open new tab and run:
'rosservice call /rrt_star_planning "{start_x: -50.0, start_y: -50.0, end_x: 50.0, end_y: 50.0, radius: 5.0, obstacle_num: 40}"'
