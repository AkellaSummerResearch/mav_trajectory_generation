
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include "../HelperFunctions/QuatRotEuler.h"
#include "../HelperFunctions/helper.h"
#include "nav_msgs/Path.h"
#include "mav_trajectory_generation_ros/PVAJS.h"
#include "mav_trajectory_generation_ros/PVAJS_array.h"

void waypoint2vertex_minSnap(const nav_msgs::Path Waypoints,
	const int dimension,
	const int derivative_to_optimize,
	mav_trajectory_generation::Vertex::Vector *vertex);

void trajectory2waypoint(
	const mav_trajectory_generation::Trajectory trajectory,
	const double dt,
	nav_msgs::Path *Waypoints,
  mav_trajectory_generation_ros::PVAJS_array *flatStates);

//Compute minimum snap trajectory and return minSnap cost
double solveMinSnap(
	const mav_trajectory_generation::Vertex::Vector vertices,
	const Eigen::VectorXd segment_times,
	const int dimension, 
	const int derivative_to_optimize,
	const double dt,
	mav_trajectory_generation::Trajectory *trajectory);