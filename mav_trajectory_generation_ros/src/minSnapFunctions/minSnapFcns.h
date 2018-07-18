
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include "../HelperFunctions/QuatRotEuler.h"
#include "../HelperFunctions/helper.h"
#include "nav_msgs/Path.h"
#include "mav_trajectory_generation_ros/PVAJS.h"
#include "mav_trajectory_generation_ros/PVAJS_array.h"

void waypoint2vertex_minSnap(const nav_msgs::Path &Waypoints,
	const int &dimension,
	mav_trajectory_generation::Vertex::Vector *vertex);

void yaw2vertex_minAcc(const nav_msgs::Path &Waypoints,
  mav_trajectory_generation::Vertex::Vector *vertex);

void trajectory2waypoint(
  const mav_trajectory_generation::Trajectory &wp_trajectory,
  const double &dt,
  mav_trajectory_generation_ros::PVAJS_array *flatStates);

void trajectory2waypoint(
  const mav_trajectory_generation::Trajectory &wp_trajectory,
  const mav_trajectory_generation::Trajectory &yaw_trajectory,
  const double &dt,
  mav_trajectory_generation_ros::PVAJS_array *flatStates);

void yawTrajectory2waypoint(
  const mav_trajectory_generation::Trajectory &yaw_trajectory,
  const double &dt,
  mav_trajectory_generation_ros::PVAJS_array *flatStates);

//Compute minimum snap trajectory and return minSnap cost
double solveMinSnap(
	const mav_trajectory_generation::Vertex::Vector &vertices,
	const Eigen::VectorXd &segment_times,
	const int &dimension, 
	mav_trajectory_generation::Trajectory *trajectory);

double solveMinAcceleration(
  const mav_trajectory_generation::Vertex::Vector &vertices,
  const Eigen::VectorXd &segment_times,
  const int &dimension, 
  mav_trajectory_generation::Trajectory *trajectory);

double solveMinAcceleration(
  const mav_trajectory_generation::Vertex::Vector &vertices,
  const std::vector<double> &segment_times,
  const int &dimension, 
  mav_trajectory_generation::Trajectory *trajectory);

//Compute minimum snap trajectory with optimal segments and return minSnap cost
double solveMinSnapGradDescent(
  const mav_trajectory_generation::Vertex::Vector &vertices,
  const int &dimension, 
  Eigen::VectorXd &segment_times,
  mav_trajectory_generation::Trajectory *trajectory,
  Eigen::VectorXd *segment_times_out);