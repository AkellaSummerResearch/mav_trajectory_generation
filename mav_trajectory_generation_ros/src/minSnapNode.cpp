#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include "mav_trajectory_generation_ros/minSnapStamped.h"
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "minSnapFunctions/minSnapFcns.h"
#include "mav_trajectory_generation_ros/PVAJS.h"
#include "mav_trajectory_generation_ros/PVAJS_array.h"


#include <sstream>

bool minSnapOptTimeService(mav_trajectory_generation_ros::minSnapStamped::Request  &req,
                           mav_trajectory_generation_ros::minSnapStamped::Response &res)
{
  // omp_set_num_threads(16);
  // int n_cores = Eigen::nbThreads( );
  // ROS_INFO("Number of cores: %d",n_cores);
  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();

  // //Declare initial variables
  const int n_w = req.Waypoints.poses.size(); //Number of waypoints
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  const double dt = 0.01; //Output sampling period
  double cost;

  //Get waypoints into vertex
  mav_trajectory_generation::Vertex::Vector vertices;
  waypoint2vertex_minSnap(req.Waypoints, dimension,
                          derivative_to_optimize, &vertices);


  //Get segment times from client request
  Eigen::VectorXd segment_times = Eigen::MatrixXd::Zero(n_w-1,1);
  int diff_time;
  for (int i = 1; i < n_w; i++){
    diff_time = (req.Waypoints.poses[i].header.stamp - req.Waypoints.poses[i-1].header.stamp).toSec();
    segment_times(i-1) = diff_time;
  }

  //Get solution for minimum snap problem
  mav_trajectory_generation::Trajectory trajectory;
  cost = solveMinSnap(vertices, segment_times, dimension, 
                      derivative_to_optimize, dt, &trajectory);

  //Calculate gradient
  int m = n_w - 1;
  double h = 0.00001;  //Small step for time gradient
  double costNew;
  Eigen::VectorXd g = (-1.0/(m-1.0))*Eigen::MatrixXd::Ones(m,1);
  Eigen::VectorXd g_i = g;
  Eigen::VectorXd gradF = Eigen::MatrixXd::Zero(m,1);
  Eigen::VectorXd segment_times_new = segment_times;
  for(int i = 0; i < m; i++){
    g_i = g;
    g_i[i] = 1;
    segment_times_new = segment_times + h*g_i;
    costNew = solveMinSnap(vertices, segment_times, dimension, 
                      derivative_to_optimize, dt, &trajectory);
    gradF(i) = (costNew - cost)/h;

  }


  //Sample trajectory
  nav_msgs::Path Waypoints;
  mav_trajectory_generation_ros::PVAJS_array flatStates;
  trajectory2waypoint(trajectory, dt, &Waypoints, &flatStates);

  //Output
  res.flatStates = flatStates;
  res.output = Waypoints;
  
  //Calculate solution time
  ros::Duration SolverTime = ros::Time::now() - t0;
  ROS_INFO("Number of points: %d\tCalculation time: %f\tCost: %f",
            n_w, SolverTime.toSec(), cost);

//   return true;
}

bool minSnapService(mav_trajectory_generation_ros::minSnapStamped::Request  &req,
                    mav_trajectory_generation_ros::minSnapStamped::Response &res)
{

  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();

  // //Declare initial variables
  const int n_w = req.Waypoints.poses.size(); //Number of waypoints
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  const double dt = 0.01; //Output sampling period
  double cost;

  //Get waypoints into vertex
  mav_trajectory_generation::Vertex::Vector vertices;
  waypoint2vertex_minSnap(req.Waypoints, dimension,
                          derivative_to_optimize, &vertices);


  //Get segment times from client request
  Eigen::VectorXd segment_times = Eigen::MatrixXd::Zero(n_w-1,1);
  int diff_time;
  for (int i = 1; i < n_w; i++){
    diff_time = (req.Waypoints.poses[i].header.stamp - req.Waypoints.poses[i-1].header.stamp).toSec();
    segment_times(i-1) = diff_time;
  }

  //Get solution for minimum snap problem
  mav_trajectory_generation::Trajectory trajectory;
  cost = solveMinSnap(vertices, segment_times, dimension, 
                      derivative_to_optimize, dt, &trajectory);

  //Sample trajectory
  nav_msgs::Path Waypoints;
  mav_trajectory_generation_ros::PVAJS_array flatStates;
  trajectory2waypoint(trajectory, dt, &Waypoints, &flatStates);

  //Output
  res.flatStates = flatStates;
  res.output = Waypoints;
  
  //Calculate solution time
  ros::Duration SolverTime = ros::Time::now() - t0;
  ROS_INFO("Number of points: %d\tCalculation time: %f\tCost: %f",
            n_w, SolverTime.toSec(), cost);

//   return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  //ROS initialization
  ros::NodeHandle n;

  //Publishers
  ros::Publisher Waypoint_pub = n.advertise<nav_msgs::Path>("Waypoints", 1000);

  //Services
  ros::ServiceServer minSnap_Srv = n.advertiseService("/minSnap", minSnapService);


  ros::Rate loop_rate(1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok())
  {

    // Waypoint_pub.publish(Waypoints);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}