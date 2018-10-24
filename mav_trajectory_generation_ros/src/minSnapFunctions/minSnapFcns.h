
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include "../HelperFunctions/QuatRotEuler.h"
#include "../HelperFunctions/helper.h"
#include "nav_msgs/Path.h"
#include "mg_msgs/PVAJS.h"
#include "mg_msgs/PVAJS_array.h"
#include "mg_msgs/PVAJ_request.h"
#include "mg_msgs/minSnapWpStamped.h"
#include "mg_msgs/minSnapWpPVAJ.h"

void waypoint2vertex_minSnap(const nav_msgs::Path &Waypoints,
	const int &dimension,
	mav_trajectory_generation::Vertex::Vector *vertex);

void waypoint2vertex_minSnap(const std::vector<mg_msgs::PVAJ_request> &PVAJyaw,
  const int &dimension,
  mav_trajectory_generation::Vertex::Vector *vertex);

void yaw2vertex_minAcc(const nav_msgs::Path &Waypoints,
  mav_trajectory_generation::Vertex::Vector *vertex);

void yaw2vertex_minAcc(const std::vector<mg_msgs::PVAJ_request> &PVAJyaw,
  mav_trajectory_generation::Vertex::Vector *vertex);

void trajectory2waypoint(
  const mav_trajectory_generation::Trajectory &wp_trajectory,
  const double &dt,
  mg_msgs::PVAJS_array *flatStates,
  std::vector<maxValues> *maxValSegments,
  maxValues *maxValTraj);

void trajectory2waypoint(
  const mav_trajectory_generation::Trajectory &wp_trajectory,
  const mav_trajectory_generation::Trajectory &yaw_trajectory,
  const double &dt,
  mg_msgs::PVAJS_array *flatStates,
  std::vector<maxValues> *maxValSegments,
  maxValues *maxValTraj);

void yawTrajectory2waypoint(
  const mav_trajectory_generation::Trajectory &yaw_trajectory,
  const double &dt,
  mg_msgs::PVAJS_array *flatStates);

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

// Solve minimum snap trajectory with optimal time segments using nlopt
void SolveNlopt(const std::vector<double> &segment_times, 
                const mav_trajectory_generation::Vertex::Vector &wp_vertices,
                const mav_trajectory_generation::Vertex::Vector &yaw_vertices,
                const double &dt, const uint &n_w,
                const double &max_vel, const double &max_acc, const double &max_jerk,
                mg_msgs::minSnapWpStamped::Response *res);

// Same as above, but with different template for the output
void SolveNlopt(const std::vector<double> &segment_times, 
                const mav_trajectory_generation::Vertex::Vector &wp_vertices,
                const mav_trajectory_generation::Vertex::Vector &yaw_vertices,
                const double &dt, const uint &n_w,
                const double &max_vel, const double &max_acc, const double &max_jerk,
                mg_msgs::minSnapWpPVAJ::Response *res);