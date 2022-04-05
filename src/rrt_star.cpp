 /*********************************************************************
 * Author: Elbera Allozi 
 * 
 * Contact info:
 * Email: alouzi@itu.edu.tr
 *        elbera97@hotmail.com
 * 
 * Github: https://github.com/Elbera97/ros_rrt_star
 * 
 * Description: This is an OMPL RRT* algorithm ROS node implementation 
 *********************************************************************/

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros_rrt_star/rrt_star_planning.h"
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Cost.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/spaces/RealVectorControlSpace.h> 
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
  
namespace ob = ompl::base;
namespace og = ompl::geometric;

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, double length_, double width_, int obstacle_num_, float radius_) :
        ob::StateValidityChecker(si), length{length_}, width{width_}, obstacle_num{obstacle_num_}, radius{radius_} {}
    bool isValid(const ob::State* state) const;
    double clearance(const ob::State* state, int NumObstacle) const;
    double clearance(const ob::State* state, int NumObstacle, bool mapborder) const;
    void fillRandomMarkers();
    void fillMapMarkers();
    void fillGoalMarkers();
    std::vector<visualization_msgs::Marker> getMarkers(){return Markers;}
    void set_private_data(double length_, double width_, int obstacle_num_, float radius_, float start_point_[2], float end_point_[2])
    {length = length_; width = width_; obstacle_num = obstacle_num_; radius = radius_; 
    start_point[0] = start_point_[0]; end_point[0] = end_point_[0];
    start_point[1] = start_point_[1]; end_point[1] = end_point_[1];}

private:
    std::vector<double> obstacle_x;
    std::vector<double> obstacle_y;
    std::vector<visualization_msgs::Marker> Markers;
    float start_point[2];
    float end_point[2];
    float radius;
    double length;
    double width;
    int obstacle_num;
};

bool ValidityChecker::isValid(const ob::State* state) const
{   
    // Check for collision with random obstacles
    for (int i=0; i < obstacle_num; i++)
    {
        if (this->clearance(state,i) < 0.0)
        {return false;}
    }
    
    // Check for collision with map borders
    for (int i=obstacle_num; i < obstacle_num + 4; i++)
    {
        if (this->clearance(state,i,true) < 0.0)
        {return false;}
    }
    
    // if no collision retun return true
    return true;
}

// Returns the distance from the given state's position to the
// boundary of the circular obstacle.
double ValidityChecker::clearance(const ob::State* state, int NumObstacle) const
{
    // Extract the robot's (x,y) position from its state
    double x = state ->as < ob :: SE2StateSpace :: StateType >() -> getX ();
    double y = state ->as < ob :: SE2StateSpace :: StateType >() -> getY ();

    // Distance formula between two points, offset by the circle's
    // radius
    return sqrt((x-obstacle_x[NumObstacle])*(x-obstacle_x[NumObstacle]) +
     (y-obstacle_y[NumObstacle])*(y-obstacle_y[NumObstacle])) - radius;
}

// Checks if the given state's position is out of the map borders 
double ValidityChecker::clearance(const ob::State* state, int NumObstacle, bool mapborder) const
{
    // Extract the robot's (x,y) position from its state
    double x = state ->as < ob :: SE2StateSpace :: StateType >() -> getX ();
    double y = state ->as < ob :: SE2StateSpace :: StateType >() -> getY ();

    if ( abs(obstacle_x[NumObstacle]) > 0.0 && abs(obstacle_x[NumObstacle])-abs(x) < 0.0 && mapborder == true)
    { return -1;}    
    
    if ( abs(obstacle_y[NumObstacle]) > 0.0 && abs(obstacle_y[NumObstacle])-abs(y) < 0.0 && mapborder == true)
    { return -1;}

    // if the given state's position is in map borders return 1
    return 1;
}

// Generates random obstacles and constructs visualization_msgs::Marker ROS message
void ValidityChecker::fillRandomMarkers()
{
    srand(time(nullptr));
    int max_x = width - (2*radius);
    int max_y = length - (2*radius);

    for (int i = 0; i < obstacle_num; i++)
    {
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // Set the marker action.  Options are ADD, DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "obstacles";
        marker.id = i;

        // Set the marker type: CUBE, SPHERE, ARROW and CYLINDER
        marker.type = visualization_msgs::Marker::CYLINDER;

        // Create random points
        double random_x = (rand() % max_x) - max_x/2; 
        obstacle_x.push_back(random_x);
        double random_y =(rand() % max_y) - max_y/2; 
        obstacle_y.push_back(random_y);

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = random_x;
        marker.pose.position.y = random_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 2*radius;
        marker.scale.y = 2*radius;
        marker.scale.z = 0.1;
    
        // Set the color
        marker.color.r = 1.0f;
        marker.color.g = 0.3f;
        marker.color.b = 0.1f;
        marker.color.a = 1.0;

        //Set marker lifetime
        marker.lifetime = ros::Duration();

        Markers.push_back(marker);
    }
}

// Generates map obstacles and constructs visualization_msgs::Marker ROS message
void ValidityChecker::fillMapMarkers()
{
    double a = (width/2)+0.5, b = (length/2)+0.5;
    double Xobstacle [4]= {0, a-0.5, 0, -a+0.5}, Yobstacle [4]= {b-0.5 , 0 , -b+0.5, 0};
    double Xposition [4]= {0, a, 0, -a}, Yposition [4]= {b , 0 , -b, 0};
    double Xscale [4]= {width, 1, -width, 1}, Yscale [4]= {1 , length , 1, -length};
    
    for (int i = 0; i < 4; i++)
    {
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // Set the marker action.  Options are ADD, DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "map borders";
        marker.id = i;

        // Set the marker type: CUBE, SPHERE, ARROW and CYLINDER
        marker.type = visualization_msgs::Marker::CUBE;

        obstacle_x.push_back(Xobstacle[i]);
        obstacle_y.push_back(Yobstacle[i]);

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = Xposition[i];
        marker.pose.position.y = Yposition[i];
        marker.pose.position.z = 0.5;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = Xscale[i];
        marker.scale.y = Yscale[i];
        marker.scale.z = 1;
    
        // Set the color
        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        //Set marker lifetime
        marker.lifetime = ros::Duration();

        Markers.push_back(marker);
    }
}

// constructs visualization_msgs::Marker ROS message for Start and Goal markers
void ValidityChecker::fillGoalMarkers()
{
    float Xposition[2] = {start_point[0],end_point[0]};
    float Yposition[2] = {start_point[1],end_point[1]}; 

    for (int i = 0; i < 2; i++)
    {
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // Set the marker action.  Options are ADD, DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "start and goal";
        marker.id = i;

        // Set the marker type: CUBE, SPHERE, ARROW and CYLINDER
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = Xposition[i];
        marker.pose.position.y = Yposition[i];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 3;
        marker.scale.y = 3;
        marker.scale.z = 0.1;
    
        // Set the color for start point (black)
        if (i == 0)
        {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;  
        }

        // Set the color for goal point (yellow)
        if (i == 1)
        {
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;  
        }


        //Set marker lifetime
        marker.lifetime = ros::Duration();

        Markers.push_back(marker);
    }
}

class rrt_star
{
public:
    rrt_star(double length_, double width_,double rrtstar_cost_, double termination_time_) :
        length{length_}, width{width_}, rrtstar_cost{rrtstar_cost_}, termination_time{termination_time_} {}
    bool run_planning(ros_rrt_star::rrt_star_planning::Request &req, ros_rrt_star::rrt_star_planning::Response &res);
    void planning();
    void path_initializer(){ros_path.header.frame_id = "map";}
    nav_msgs::Path get_ros_path(){return ros_path;}
    std::vector<visualization_msgs::Marker> getMarkers(){return Markers;}

private:
    std::vector<visualization_msgs::Marker> Markers;
    nav_msgs::Path ros_path;
    ob::PlannerStatus solved;
    ros::Time Service_start_time;
    ros::Time Service_end_time;

    double length;
    double width;
    double rrtstar_cost;
    double termination_time;
    float radius;
    int obstacle_num;
    float start_point[2];
    float end_point[2];
    bool goal_reached;
    void get_xy_theta(const ob::State *state, float &x, float &y, float &theta);
};

// Gets requests from ROS service
// Calls planning()
// Calculates planning time
// Sends response
bool rrt_star::run_planning(ros_rrt_star::rrt_star_planning::Request &req, ros_rrt_star::rrt_star_planning::Response &res)
{   
    Service_start_time = ros::Time::now();
    start_point[0] = req.start_x;
    start_point[1] = req.start_y;
    end_point[0] = req.end_x;
    end_point[1] = req.end_y;
    obstacle_num = req.obstacle_num;
    radius = req.radius;
    planning();
    if(solved && goal_reached == true)
    {   
        res.is_path_planned = true;
        Service_end_time = ros::Time::now();
        res.calculation_time_ms = (Service_end_time-Service_start_time).toNSec() * 1e-6;
    }
    else
    {   
        res.is_path_planned = false;
        res.calculation_time_ms = 0.0;
    }

    return true;
} 


void rrt_star::planning() 
{   
    // clear poses from the last planning request 
    ros_path.poses.clear();
    
    // construct SE2 state space => (x,y,yaw)
    auto space(std::make_shared<ob::SE2StateSpace>());
  
    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(1,-length/2); //set min  y
    bounds.setHigh(1,length/2); //set max y
    bounds.setLow(0,-width/2);  //set min  x
    bounds.setHigh(0,width/2);  //set max x
    
    space->setBounds(bounds);
  
    // construct an instance of space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    ValidityChecker *ValidityCheckerOPJ = new ValidityChecker(si,length,width,obstacle_num,radius);
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(ValidityCheckerOPJ));  
    
    ValidityCheckerOPJ->set_private_data(length,width,obstacle_num,radius,start_point,end_point);
    ValidityCheckerOPJ->fillRandomMarkers();
    ValidityCheckerOPJ->fillMapMarkers();
    ValidityCheckerOPJ->fillGoalMarkers();
    
    // set start point
    ob::ScopedState<> start(space);
    start[0] = start_point[0];
    start[1] = start_point[1];
    //start[2] = 45 * M_PI / 180.0;

    // set goal point
    ob::ScopedState<> goal(space);
    goal[0] = end_point[0];
    goal[1] = end_point[1];
    //goal[2] = 180 * M_PI / 180.0;

    // create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    
    // create an optimization instance
    ob::OptimizationObjectivePtr ooPtr(new ob::PathLengthOptimizationObjective(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // set cost
    ob::Cost cost{rrtstar_cost}; 
    ooPtr->setCostThreshold(cost);
    pdef->setOptimizationObjective(ooPtr);
  
    // create a RRT* planner for the defined space
    auto planner(std::make_shared<og::RRTstar>(si));
  
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
  
    // perform setup steps for the planner
    planner->setup();
  
    // print the settings for this space
    si->printSettings(std::cout);
  
    // print the problem settings
    pdef->print(std::cout);
  
    // attempt to solve the problem within 'termination_time' seconds of planning time
    solved = planner->ob::Planner::solve(termination_time);

    geometry_msgs::PoseStamped ps;
    tf2::Quaternion myQuaternion;
  
    if (solved)
    {        
        std::cout << "Found solution:" << std::endl;

        // get planned path
        ob::PathPtr path = pdef->getSolutionPath();
        
        // print the path to screen 
        path->print(std::cout);

        // convert path to individual states 
        og::PathGeometric& result_path = static_cast<og::PathGeometric&>(*path);
        std::vector<ob::State*>& result_states = result_path.getStates();

        // Conversion loop from states to ROS message:
        for (std::vector<ob::State*>::iterator it = result_states.begin(); it != result_states.end(); ++it)
        {               
            // Get the data from the state
            float x, y, theta;
            get_xy_theta(*it, x, y, theta);

            // Create this quaternion from roll/pitch/yaw (in radians)
            myQuaternion.setRPY( 0, 0, theta );
            myQuaternion.normalize();
            
            // Place data into the pose:
            ps.header.stamp = ros::Time::now();
            ps.header.frame_id = "map";
            ps.pose.position.x = x;
            ps.pose.position.y = y; 
            ps.pose.position.z = 0.0;
            ps.pose.orientation.x = myQuaternion[0];
            ps.pose.orientation.y = myQuaternion[1];
            ps.pose.orientation.z = myQuaternion[2];
            ps.pose.orientation.w = myQuaternion[3];
            ros_path.header.stamp = ros::Time::now();
            ros_path.header.frame_id = "map";
            ros_path.poses.push_back(ps);

            // check if the last pose of the path is indeed the goal pose
            if(it == result_states.end()-1)
            {
                if(x == goal[0] && y == goal[1]) {goal_reached = true;}
                else{goal_reached = false;}
            }
        }
    }
    else
    {   
        std::cout << "No solution found" << std::endl;  
        // if no solution found set an empty path ROS message with 'map' frame_id to avoid Rviz errors
        ps.header.frame_id = "map";
        ros_path.header.frame_id = "map";
        ros_path.poses.push_back(ps);
    }

    // get martkers from ValidityChecker class 
    Markers = ValidityCheckerOPJ->getMarkers(); 
} 

void rrt_star::get_xy_theta(const ob::State *state, float &x, float &y, float &theta)
{
    x = state ->as < ob :: SE2StateSpace :: StateType >() -> getX ();
    y = state ->as < ob :: SE2StateSpace :: StateType >() -> getY ();
    theta = state ->as < ob :: SE2StateSpace :: StateType >() -> getYaw ();
}


int main(int argc, char **argv)
{    
    ros::init(argc, argv, "rrt_star_node");
    ros::NodeHandle node;
	ros::NodeHandle nodeParam("~");

    double length,width,rrtstar_cost,termination_time;
    nodeParam.getParam("length", length);
    nodeParam.getParam("width", width);
    nodeParam.getParam("rrtstar_cost", rrtstar_cost);
    nodeParam.getParam("termination_time", termination_time);

    std::cout << " " << std::endl;
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    std::cout << "map length (y direction): " << length << std::endl;
    std::cout << "map width  (x direction): " << width << std::endl;
    std::cout << "rrtstar_cost:             " << rrtstar_cost << std::endl;
    std::cout << "termination_time:         " << termination_time << std::endl;


    rrt_star rrt_starOPJ{length,width,rrtstar_cost,termination_time};
    rrt_starOPJ.path_initializer();

    ros::ServiceServer service = node.advertiseService("rrt_star_planning",&rrt_star::run_planning,&rrt_starOPJ);

	ros::Rate loop_rate(10);
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 0.1);
    ros::Publisher path_pub = node.advertise<nav_msgs::Path>("nav_msgs/Path", 0.1);

    std::vector<visualization_msgs::Marker> Markers;

    while (ros::ok())
    {   
        // Publish markers
        Markers = rrt_starOPJ.getMarkers();
        for (int i=0; i < Markers.size(); i++)
        {marker_pub.publish(Markers[i]);}
        
        // Publish path        
        path_pub.publish(rrt_starOPJ.get_ros_path());

    	ros::spinOnce();
        loop_rate.sleep();
  	}

	return 0;
}
