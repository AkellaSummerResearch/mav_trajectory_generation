
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
	nav_msgs::Path *Waypoints,
  mav_trajectory_generation_ros::PVAJS_array *flatStates){

  
  //Get whole trajectory
  mav_msgs::EigenTrajectoryPoint::Vector states;
  bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, dt, &states);
  if (!success) {
    ROS_INFO("Error while sampling trajectory!");
    return;
  }

  //Populate waypoints to send back
  geometry_msgs::PoseStamped Pos;
  mav_trajectory_generation_ros::PVAJS flatState;

  mav_trajectory_generation::EigenTrajectoryPoint2PVAJS_array(states, flatStates);
  for (int i = 0; i < states.size(); i++){
    Pos.pose.position = Eigen2Point(states[i].position_W);
    Pos.header.stamp = ros::Time().fromNSec(states[i].time_from_start_ns);
    Waypoints->poses.push_back(Pos);
   }
}

double solveMinSnap(
	const mav_trajectory_generation::Vertex::Vector vertices,
	const Eigen::VectorXd segment_times,
	const int dimension, 
	const int derivative_to_optimize,
	const double dt,
	mav_trajectory_generation::Trajectory *trajectory){

  //Convert segment times to the appropriate type
  std::vector<double> stdSegment_times;
  eigenVectorXd2stdVector(segment_times, &stdSegment_times);

  //Solve optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, stdSegment_times, derivative_to_optimize);
  opt.solveLinear();

  //Get segments
  mav_trajectory_generation::Segment::Vector segments;
  opt.getSegments(&segments);

  //Get trajectory
  // mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(trajectory);

  return opt.computeCost();
}