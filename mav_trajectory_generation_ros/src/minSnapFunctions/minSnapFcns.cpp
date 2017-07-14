
#include "minSnapFcns.h"

void waypoint2vertex_minSnap(const nav_msgs::Path Waypoints,
	const int dimension,
	const int derivative_to_optimize,
	mav_trajectory_generation::Vertex::Vector *vertex){

  //Declare initial variables
  int n_w = Waypoints.poses.size();
  mav_trajectory_generation::Vertex start_end(dimension), middle(dimension);


  //Populate waypoints
  Eigen::Vector3d Point;
  for(int i = 0; i < n_w; i++){

    Point << Waypoints.poses[i].pose.position.x,
             Waypoints.poses[i].pose.position.y,
             Waypoints.poses[i].pose.position.z;

    if ((i == 0) || (i == n_w -1)){
      start_end.makeStartOrEnd(Point, derivative_to_optimize);
      vertex->push_back(start_end);
    }
    else{
      middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Point);
      vertex->push_back(middle);
    }

  }
}

void trajectory2waypoint(
	const mav_trajectory_generation::Trajectory trajectory,
	const double dt,
	nav_msgs::Path *Waypoints){

  //Sample trajectory
  double t_start = 0.0;
  double t_end = trajectory.getMaxTime();
  
  int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
  std::vector<Eigen::VectorXd> result;
  std::vector<double> sampling_times; // Optional.
  trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);


  //Populate waypoints to send back
  geometry_msgs::PoseStamped Pos;
  Eigen::VectorXd Waypoint;
  for (int i = 0; i < result.size(); i++){
    Waypoint = result[i];
    Pos.pose.position = SetPoint(Waypoint[0], Waypoint[1], Waypoint[2]);
    Pos.header.stamp = ros::Time().fromSec(sampling_times[i]);
    Waypoints->poses.push_back(Pos);
   }
}

double solveMinSnap(
	const mav_trajectory_generation::Vertex::Vector vertices,
	const std::vector<double> segment_times,
	const int dimension, 
	const int derivative_to_optimize,
	const double dt,
	mav_trajectory_generation::Trajectory *trajectory){

  //Solve optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  //Get segments
  mav_trajectory_generation::Segment::Vector segments;
  opt.getSegments(&segments);

  //Get trajectory
  // mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(trajectory);

  return opt.computeCost();
}