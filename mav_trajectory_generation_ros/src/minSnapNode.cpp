#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/extremum.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "minSnapFunctions/minSnapFcns.h"
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/structs.h>
#include "mg_msgs/minSnapWpStamped.h"
#include "mg_msgs/minSnapWpPVAJ.h"

#include <sstream>
#include <thread>

std::queue<waypoint_and_trajectory> wp_traj_list;
std::mutex m_wp_traj_list;

bool minSnapNloptService(mg_msgs::minSnapWpStamped::Request  &req,
                         mg_msgs::minSnapWpStamped::Response &res) {
  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();

  //Declare initial variables
  const uint n_w = req.Waypoints.poses.size(); //Number of waypoints
  double dt = req.dt_flat_states.data; //Output sampling period
  const int dimension = 3, yaw_dimension = 1;

  if (n_w <= 2) {
    ROS_WARN("[min_snap_node] Not enough waypoints! Canceling service...");
    return false;
  }
  if (dt <= 0) {
    ROS_WARN("[min_snap_node] Sampling time not well defined. Setting sampling time to dt=0.01s");
    dt = 0.01;
  }

  //Get waypoints into vertex
  mav_trajectory_generation::Vertex::Vector wp_vertices, yaw_vertices;
  waypoint2vertex_minSnap(req.Waypoints, dimension, &wp_vertices);
  yaw2vertex_minAcc(req.Waypoints, &yaw_vertices);

  //Get segment times from client request
  std::vector<double> segment_times;
  double diff_time;
  for (int i = 1; i < n_w; i++){
    diff_time = (req.Waypoints.poses[i].header.stamp - req.Waypoints.poses[i-1].header.stamp).toSec();
    segment_times.push_back(diff_time);
    // std::cout << "segment time: " << segment_times[i-1] << std::endl;
  }

  double max_vel = 0, max_acc = 0, max_jerk = 0;
  SolveNlopt(segment_times, wp_vertices, yaw_vertices, dt,
             n_w, max_vel, max_acc, max_jerk, &res);

  // Add to list for Rviz publishing
  m_wp_traj_list.lock();
    wp_traj_list.push(waypoint_and_trajectory(req.Waypoints, res.flatStates));
  m_wp_traj_list.unlock();

  //Calculate solution time
  ros::Duration SolverTime = ros::Time::now() - t0;
  ROS_INFO("Number of points: %d\tCalculation time: %f\tCost: %f",
            n_w, SolverTime.toSec(), res.cost.data);
  
  return true;
}

bool minSnapNloptExtendedService(mg_msgs::minSnapWpPVAJ::Request  &req,
                                 mg_msgs::minSnapWpPVAJ::Response &res) {
  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();

  //Declare initial variables
  const uint n_w = req.PVAJ_array.size(); //Number of waypoints
  double dt = req.dt_flat_states.data; //Output sampling period
  const int dimension = 3, yaw_dimension = 1;

  if (n_w <= 2) {
    ROS_WARN("[min_snap_node] Not enough waypoints! Canceling service...");
    return false;
  }
  if (dt <= 0) {
    ROS_WARN("[min_snap_node] Sampling time not well defined. Setting sampling time to dt=0.01s");
    dt = 0.01;
  }

  //Get waypoints into vertex
  mav_trajectory_generation::Vertex::Vector wp_vertices, yaw_vertices;
  waypoint2vertex_minSnap(req.PVAJ_array, dimension, &wp_vertices);
  yaw2vertex_minAcc(req.PVAJ_array, &yaw_vertices);

  //Get segment times from client request
  std::vector<double> segment_times;
  double diff_time;
  for (int i = 1; i < n_w; i++){
    diff_time = req.PVAJ_array[i].time - req.PVAJ_array[i-1].time;
    segment_times.push_back(diff_time);
    // std::cout << "segment time: " << segment_times[i-1] << std::endl;
  }

  double max_vel = req.max_vel, max_acc = req.max_acc, max_jerk = req.max_jerk;
  SolveNlopt(segment_times, wp_vertices, yaw_vertices, dt,
             n_w, max_vel, max_acc, max_jerk, &res);

  // Add to list for Rviz publishing
  m_wp_traj_list.lock();
    wp_traj_list.push(waypoint_and_trajectory(req.PVAJ_array, res.flatStates));
  m_wp_traj_list.unlock();

  //Calculate solution time
  ros::Duration SolverTime = ros::Time::now() - t0;
  ROS_INFO("Number of points: %d\tCalculation time: %f\tCost: %f",
            n_w, SolverTime.toSec(), res.cost.data);
  
  return true;
}

bool minSnapExtendedOptTimeService(mg_msgs::minSnapWpPVAJ::Request  &req,
                                   mg_msgs::minSnapWpPVAJ::Response &res) {
  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();

  // //Declare initial variables
  const int n_w = req.PVAJ_array.size(); //Number of waypoints
  double dt = req.dt_flat_states.data; //Output sampling period
  const int dimension = 3, yaw_dimension = 1;
  double cost, yaw_cost;

  if (n_w <= 2) {
    ROS_WARN("[min_snap_node] Not enough waypoints! Canceling service...");
    return false;
  }
  if (dt <= 0) {
    ROS_WARN("[min_snap_node] Sampling time not well defined. Setting sampling time to dt=0.01s");
    dt = 0.01;
  }

  //Get waypoints into vertex
  mav_trajectory_generation::Vertex::Vector wp_vertices, yaw_vertices;
  waypoint2vertex_minSnap(req.PVAJ_array, dimension, &wp_vertices);
  yaw2vertex_minAcc(req.PVAJ_array, &yaw_vertices);

  //Get segment times from client request
  Eigen::VectorXd segment_times = Eigen::MatrixXd::Zero(n_w-1,1);
  double diff_time;
  for (int i = 1; i < n_w; i++){
    diff_time = req.PVAJ_array[i].time - req.PVAJ_array[i-1].time;
    segment_times(i-1) = diff_time;
  }

  //Get solution for minimum snap problem with optimal segment times
  mav_trajectory_generation::Trajectory trajectory_wp, trajectory_yaw;
  Eigen::VectorXd best_segment_times;
  cost = solveMinSnapGradDescent(wp_vertices, dimension, segment_times,
                                 &trajectory_wp, &best_segment_times);

  //Sample trajectory to check if trajetory exceeds maximum constraints
  mg_msgs::PVAJS_array flatStates;
  std::vector<maxValues> maxValSegments;
  maxValues maxValTraj;
  trajectory2waypoint(trajectory_wp, dt, &flatStates, &maxValSegments, &maxValTraj);

  bool max_vel_safe = false, max_acc_safe = false, max_jerk_safe = false;
  double tolerance = 1.1;
  uint max_iter = 2, count = 0;
  while(!max_vel_safe || !max_acc_safe || !max_jerk_safe) {
    bool solve_again = false; // Boolean that identifies if min snap needs to be solved again
    for (uint i = 0; i < n_w-1; i++) {
      // ROS_INFO("Segment [%d]: Max vel: %4.2f\tMax acc:%4.2f\tMax Jerk: %4.2f",
      //           i+1, maxValSegments[i].max_vel, maxValSegments[i].max_acc, maxValSegments[i].max_jerk);  
      // ROS_INFO("segment time: %f", best_segment_times[i]);
      double ratio = 1.0, ratio_vel, ratio_acc, ratio_jerk;
      if((maxValSegments[i].max_vel > req.max_vel) && (req.max_vel > 0.0)) {
        double ratio_vel = maxValSegments[i].max_vel/req.max_vel;
        if (ratio_vel > ratio) {
          ratio = ratio_vel;
        }
      }
      if((maxValSegments[i].max_acc > req.max_acc) && (req.max_acc > 0.0)) {
        double ratio_acc = sqrt(maxValSegments[i].max_acc/req.max_acc);
        if (ratio_acc > ratio) {
          ratio = ratio_acc;
        }
      }
      if((maxValSegments[i].max_jerk > req.max_jerk) && (req.max_jerk > 0.0)) {
        double ratio_jerk = cbrt(maxValSegments[i].max_jerk/req.max_jerk);
        if (ratio_jerk > ratio) {
          ratio = ratio_jerk;
        }
      }
      if (ratio > 1.0) {  // Solve minimum snap again with new final time
        best_segment_times[i] = ratio*best_segment_times[i];
        solve_again = true;
      }
    }

    // If needed, solve minimum snap again with new segment times
    if (solve_again) {  
      cost = solveMinSnap(wp_vertices, best_segment_times, dimension, &trajectory_wp); 
    }

    ROS_INFO("Max vel: %3.2f\tMax acc:%3.2f\tMax Jerk: %3.2f",
              maxValTraj.max_vel, maxValTraj.max_acc, maxValTraj.max_jerk);

    // Resample trajectory (update maximum vel/acc/jerk)
    flatStates.PVAJS_array.clear();
    trajectory2waypoint(trajectory_wp, dt, &flatStates, &maxValSegments, &maxValTraj);

    if ((maxValTraj.max_vel < tolerance*req.max_vel) || (req.max_vel <= 0.0)) {
      max_vel_safe = true;
    }
    if ((maxValTraj.max_acc < tolerance*req.max_acc) || (req.max_acc <= 0.0)) {
      max_acc_safe = true;
    }
    if ((maxValTraj.max_jerk < tolerance*req.max_jerk) || (req.max_jerk <= 0.0)) {
      max_jerk_safe = true;
    }

    // Check for maximum number of iterations
    if(++count > max_iter) {
      break;
    }
  }
 
  ROS_INFO("Max vel: %3.2f\tMax acc:%3.2f\tMax Jerk: %3.2f",
              maxValTraj.max_vel, maxValTraj.max_acc, maxValTraj.max_jerk);
  ROS_INFO("Final time: %4.2f", trajectory_wp.getMaxTime());

  yaw_cost = solveMinAcceleration(yaw_vertices, best_segment_times, yaw_dimension,
                                      &trajectory_yaw);
  flatStates.PVAJS_array.clear();
  trajectory2waypoint(trajectory_wp, trajectory_yaw, dt, &flatStates, &maxValSegments, &maxValTraj);


  //Calculate solution time
  ros::Duration SolverTime = ros::Time::now() - t0;
  ROS_INFO("Number of points: %d\tCalculation time: %4.2f\tCost: %10.3f",
            n_w, SolverTime.toSec(), cost);


  //Sample trajectory
  // mg_msgs::PVAJS_array flatStatesFinal;

  // for (uint i = 0; i < n_w-1; i++) {
  //   ROS_INFO("Segment [%d]: Max vel: %4.2f\tMax acc:%4.2f\tMax Jerk: %4.2f",
  //             i+1, maxValSegments[i].max_vel, maxValSegments[i].max_acc, maxValSegments[i].max_jerk);  
  //   ROS_INFO("segment time: %f", best_segment_times[i]);
  // }
  // ROS_INFO("Max vel: %f\tMax acc:%f\tMax Jerk: %f",
  //           maxValTraj.max_vel, maxValTraj.max_acc, maxValTraj.max_jerk);

  // ROS_INFO("Max vel: %f\tMax acc:%f\tMax Jerk: %f", maxVal.max_vel, maxVal.max_acc, maxVal.max_jerk);

  // Add to list for Rviz publishing
  m_wp_traj_list.lock();
    wp_traj_list.push(waypoint_and_trajectory(req.PVAJ_array, flatStates));
  m_wp_traj_list.unlock();

  //Output
  res.flatStates = flatStates;
  res.cost.data = cost;

  std_msgs::Float32 dt_wp;
  for (int i = 0; i < n_w-1; i++){
    dt_wp.data = best_segment_times(i);
    res.dt_out.push_back(dt_wp);
  }

  return true;
}


bool minSnapOptTimeService(mg_msgs::minSnapWpStamped::Request  &req,
                           mg_msgs::minSnapWpStamped::Response &res) {
  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();

  // //Declare initial variables
  const int n_w = req.Waypoints.poses.size(); //Number of waypoints
  const int dimension = 3, yaw_dimension = 1;
  double dt = req.dt_flat_states.data; //Output sampling period
  double cost, yaw_cost;

  if (n_w <= 2) {
    ROS_WARN("[min_snap_node] Not enough waypoints! Canceling service...");
    return false;
  }
  if (dt <= 0) {
    ROS_WARN("[min_snap_node] Sampling time not well defined. Setting sampling time to dt=0.01s");
    dt = 0.01;
  }

  //Get waypoints into vertex
  mav_trajectory_generation::Vertex::Vector wp_vertices, yaw_vertices;
  waypoint2vertex_minSnap(req.Waypoints, dimension, &wp_vertices);
  yaw2vertex_minAcc(req.Waypoints, &yaw_vertices);

  //Get segment times from client request
  Eigen::VectorXd segment_times = Eigen::MatrixXd::Zero(n_w-1,1);
  double diff_time;
  for (int i = 1; i < n_w; i++){
    diff_time = (req.Waypoints.poses[i].header.stamp - req.Waypoints.poses[i-1].header.stamp).toSec();
    segment_times(i-1) = diff_time;
  }

  //Get solution for minimum snap problem with optimal segment times
  mav_trajectory_generation::Trajectory trajectory_wp, trajectory_yaw;
  Eigen::VectorXd best_segment_times;
  cost = solveMinSnapGradDescent(wp_vertices, dimension, segment_times,
                                 &trajectory_wp, &best_segment_times);
  yaw_cost = solveMinAcceleration(yaw_vertices, best_segment_times, yaw_dimension,
                                  &trajectory_yaw);

  //Calculate solution time
  ros::Duration SolverTime = ros::Time::now() - t0;
  ROS_INFO("Number of points: %d\tCalculation time: %f\tCost: %f",
            n_w, SolverTime.toSec(), cost);


  //Sample trajectory
  mg_msgs::PVAJS_array flatStates;
  std::vector<maxValues> maxValSegments;
  maxValues maxValTraj;
  trajectory2waypoint(trajectory_wp, trajectory_yaw, dt, &flatStates, &maxValSegments, &maxValTraj);

  // Add to list for Rviz publishing
  m_wp_traj_list.lock();
    wp_traj_list.push(waypoint_and_trajectory(req.Waypoints, flatStates));
  m_wp_traj_list.unlock();

  //Output
  res.flatStates = flatStates;
  res.cost.data = cost;

  std_msgs::Float32 dt_wp;
  for (int i = 0; i < n_w-1; i++){
    dt_wp.data = best_segment_times(i);
    res.dt_out.push_back(dt_wp);
  }

  return true;
}

bool minSnapService(mg_msgs::minSnapWpStamped::Request  &req,
                    mg_msgs::minSnapWpStamped::Response &res) {
  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();
  ros::Duration SolverTime;

  // //Declare initial variables
  const int n_w = req.Waypoints.poses.size(); //Number of waypoints
  const int dimension = 3, yaw_dimension = 1;
  double dt = req.dt_flat_states.data; //Output sampling period
  double cost, yaw_cost;

  if (n_w <= 2) {
    ROS_WARN("[min_snap_node] Not enough waypoints! Canceling service...");
    return false;
  }
  if (dt <= 0) {
    ROS_WARN("[min_snap_node] Sampling time not well defined. Setting sampling time to dt=0.01s");
    dt = 0.01;
  }

  //Get waypoints into vertex
  mav_trajectory_generation::Vertex::Vector wp_vertices, yaw_vertices;
  waypoint2vertex_minSnap(req.Waypoints, dimension, &wp_vertices);
  yaw2vertex_minAcc(req.Waypoints, &yaw_vertices);

  //Get segment times from client request
  Eigen::VectorXd segment_times = Eigen::MatrixXd::Zero(n_w-1,1);
  double diff_time;
  for (int i = 1; i < n_w; i++){
    diff_time = (req.Waypoints.poses[i].header.stamp - req.Waypoints.poses[i-1].header.stamp).toSec();
    segment_times(i-1) = diff_time;
  }

  //Get solution for minimum snap problem
  mav_trajectory_generation::Trajectory trajectory_wp, trajectory_yaw;
  cost = solveMinSnap(wp_vertices, segment_times, dimension, &trajectory_wp);
  yaw_cost = solveMinAcceleration(yaw_vertices, segment_times, yaw_dimension,
                                  &trajectory_yaw);

  //Calculate solution time
  SolverTime = ros::Time::now() - t0;
  ROS_INFO("[min_snap_node] Number of points: %d\tCalculation time: %f\tCost: %f",
            n_w, SolverTime.toSec(), cost);

  //Sample trajectory
  mg_msgs::PVAJS_array flatStates;
  std::vector<maxValues> maxValSegments;
  maxValues maxValTraj;
  trajectory2waypoint(trajectory_wp, trajectory_yaw, dt, &flatStates, &maxValSegments, &maxValTraj);
  // Add to list for Rviz publishing
  m_wp_traj_list.lock();
    wp_traj_list.push(waypoint_and_trajectory(req.Waypoints, flatStates));
  m_wp_traj_list.unlock();
  //Output
  res.flatStates = flatStates;
  res.cost.data = cost;

  return true;
}

void RvizPubThread(ros::Publisher *pathMarker_pub, ros::Publisher *wpMarker_pub) {
  ROS_INFO("[min_snap_node] Rviz publisher thread has started!");
  ros::Rate loop_rate(0.5);

  waypoint_and_trajectory wp_and_traj;
  bool new_traj = false;
  double distance = 1.0;
  std::string frame_id = "map";
  visualization_msgs::MarkerArray TrajMarkers, WaypointMarkers;

  while (ros::ok()) {
    m_wp_traj_list.lock();
      if(!wp_traj_list.empty()) {
        wp_and_traj = wp_traj_list.front();
        wp_traj_list.pop();
        new_traj = true;
      }
    m_wp_traj_list.unlock();

    // If new trajectory, create Rviz markers
    if (new_traj) {
      mav_msgs::EigenTrajectoryPoint::Vector new_states;
      if(wp_and_traj.flatStates_.PVAJS_array.size() > 0) {
        mav_trajectory_generation::PVAJS_array2EigenTrajectoryPoint(wp_and_traj.flatStates_, &new_states);
        mav_trajectory_generation::drawMavSampledTrajectory(new_states, distance, frame_id, &TrajMarkers);
      }
      mav_trajectory_generation::drawWaypoints(wp_and_traj.Waypoints_, frame_id, &WaypointMarkers);
    }

    //Publish current markers
    pathMarker_pub->publish(TrajMarkers);
    wpMarker_pub->publish(WaypointMarkers);

    loop_rate.sleep();
  }
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

  // Rviz marker publishers
  ros::Publisher pathMarker_pub = n.advertise<visualization_msgs::MarkerArray>
                                    ("path_visualization_markers", 1000);
  ros::Publisher wpMarker_pub = n.advertise<visualization_msgs::MarkerArray>
                                    ("waypoint_visualization_markers", 1000);

  // Thread for publishing incoming waypoints and outcoming trajectories into RVIz
  std::thread h_trajPub_thread_;
  h_trajPub_thread_ = std::thread(RvizPubThread, &pathMarker_pub, &wpMarker_pub);

  //Services
  ros::ServiceServer minSnap_Srv = n.advertiseService("minSnap", minSnapService);
  ros::ServiceServer minSnap_Srv2 = n.advertiseService("minSnapOptTime", minSnapOptTimeService);
  ros::ServiceServer minSnap_Srv3 = n.advertiseService("minSnapOptTimeExtended", minSnapExtendedOptTimeService);
  ros::ServiceServer minSnap_Srv4 = n.advertiseService("minSnapNlopt", minSnapNloptService);
  ros::ServiceServer minSnap_Srv5 = n.advertiseService("minSnapNloptExtended", minSnapNloptExtendedService);

  ros::Rate loop_rate(50);

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