# OMPL_RRTstar_ROS
This is an OMPL RRT* algorithm ROS node implementation

# Example
rosservice call /rrt_star_planning "{start_x: -50.0, start_y: -50.0, end_x: 50.0, end_y: 50.0, radius: 5.0, obstacle_num: 40}"
