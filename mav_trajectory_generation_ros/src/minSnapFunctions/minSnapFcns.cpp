
#include "minSnapFcns.h"

void waypoint2vertex_minSnap(const nav_msgs::Path &Waypoints,
	const int &dimension,
	mav_trajectory_generation::Vertex::Vector *vertex){

  //Declare initial variables
  int n_w = Waypoints.poses.size();
  mav_trajectory_generation::Vertex start_end(dimension), middle(dimension);
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

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

void waypoint2vertex_minSnap(const std::vector<mg_msgs::PVAJ_request> &PVAJyaw,
  const int &dimension,
  mav_trajectory_generation::Vertex::Vector *vertex){

  //Declare initial variables
  int n_w = PVAJyaw.size();
  mav_trajectory_generation::Vertex start_end(dimension), middle(dimension);
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  //Populate waypoints
  Eigen::Vector3d Point, Vel, Acc, Jerk;
  for(int i = 0; i < n_w; i++){

    Point = Point2Eigen(PVAJyaw[i].Pos);
    Vel = Vector3_2Eigen(PVAJyaw[i].Vel);
    Acc = Vector3_2Eigen(PVAJyaw[i].Acc);
    Jerk = Vector3_2Eigen(PVAJyaw[i].Jerk);

    if ((i == 0) || (i == n_w -1)){
      start_end.makeStartOrEnd(Point, derivative_to_optimize);
      vertex->push_back(start_end);
    }
    else{
      if(PVAJyaw[i].use_pos) {
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Point);
      }
      if(PVAJyaw[i].use_vel) {
        middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Vel);
      }
      if(PVAJyaw[i].use_acc) {
        middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Acc);
      }
      if(PVAJyaw[i].use_jerk) {
        middle.addConstraint(mav_trajectory_generation::derivative_order::JERK, Jerk);
      }
      vertex->push_back(middle);
    }

  }
}

void yaw2vertex_minAcc(const nav_msgs::Path &Waypoints,
  mav_trajectory_generation::Vertex::Vector *vertex){

  //Declare initial variables
  int n_w = Waypoints.poses.size();
  const int dimension = 1;
  mav_trajectory_generation::Vertex start_end(dimension), middle(dimension);
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;

  //Populate yaw waypoints
  double yaw, prev_yaw;
  for(int i = 0; i < n_w; i++){

    yaw = getHeadingFromQuat(Waypoints.poses[i].pose.orientation);

    if (i > 0) {
      while (prev_yaw - yaw > M_PI) {
        yaw = yaw + 2*M_PI;
      }
      while (prev_yaw - yaw < -M_PI) {
        yaw = yaw - 2*M_PI;
      }
    }
    // std::cout << "Yaw: [" << i << "] = " << yaw << "\t\t" << prev_yaw - yaw << std::endl;

    if ((i == 0) || (i == n_w -1)){
      start_end.makeStartOrEnd(yaw, derivative_to_optimize);
      vertex->push_back(start_end);
    }
    else{
      middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, yaw);
      vertex->push_back(middle);
    }

    prev_yaw = yaw;
  }
}

void yaw2vertex_minAcc(const std::vector<mg_msgs::PVAJ_request> &PVAJyaw,
  mav_trajectory_generation::Vertex::Vector *vertex) {
  //Declare initial variables
  int n_w = PVAJyaw.size();
  const int dimension = 1;
  mav_trajectory_generation::Vertex start_end(dimension), middle(dimension);
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;

  //Populate yaw waypoints
  double yaw, prev_yaw;
  for(int i = 0; i < n_w; i++){

    yaw = PVAJyaw[i].yaw;

    if (i > 0) {
      while (prev_yaw - yaw > M_PI) {
        yaw = yaw + 2*M_PI;
      } 
      while( prev_yaw - yaw < -M_PI) {
        yaw = yaw - 2*M_PI;
      }
    }
    // std::cout << "Yaw: [" << i << "] = " << yaw << "\t\t" << prev_yaw - yaw << std::endl;

    if ((i == 0) || (i == n_w -1)){
      start_end.makeStartOrEnd(yaw, derivative_to_optimize);
      vertex->push_back(start_end);
    }
    else{
      if(PVAJyaw[i].use_yaw) {
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, yaw);
      }
      if(PVAJyaw[i].use_yaw_dot) {
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, PVAJyaw[i].yaw_dot);
      }
      vertex->push_back(middle);
    }

    prev_yaw = yaw;
  }
}

void trajectory2waypoint(
  const mav_trajectory_generation::Trajectory &wp_trajectory,
  const double &dt,
  mg_msgs::PVAJS_array *flatStates,
  std::vector<maxValues> *maxValSegments,
  maxValues *maxValTraj){

  //Get whole trajectory
  mav_msgs::EigenTrajectoryPoint::Vector states;
  bool success = mav_trajectory_generation::sampleWholeTrajectory(wp_trajectory, dt, &states, maxValSegments, maxValTraj);
  if (!success) {
    ROS_INFO("Error while sampling trajectory!");
    return;
  }

  //Populate waypoints to send back
  geometry_msgs::PoseStamped Pos;
  mg_msgs::PVAJS flatState;

  mav_trajectory_generation::EigenTrajectoryPoint2PVAJS_array(states, flatStates);
}

void trajectory2waypoint(
	const mav_trajectory_generation::Trajectory &wp_trajectory,
  const mav_trajectory_generation::Trajectory &yaw_trajectory,
	const double &dt,
  mg_msgs::PVAJS_array *flatStates,
  std::vector<maxValues> *maxValSegments,
  maxValues *maxValTraj){

  //Get whole trajectory
  mav_msgs::EigenTrajectoryPoint::Vector states;
  bool success = mav_trajectory_generation::sampleWholeTrajectory(wp_trajectory, dt, &states, maxValSegments, maxValTraj);
  if (!success) {
    ROS_INFO("Error while sampling trajectory!");
    return;
  }

  //Populate waypoints to send back
  geometry_msgs::PoseStamped Pos;
  mg_msgs::PVAJS flatState;

  mav_trajectory_generation::EigenTrajectoryPoint2PVAJS_array(states, flatStates);

  // Get yaw trajectory
  yawTrajectory2waypoint(yaw_trajectory, dt, flatStates);
}

void yawTrajectory2waypoint(
  const mav_trajectory_generation::Trajectory &yaw_trajectory,
  const double &dt,
  mg_msgs::PVAJS_array *flatStates) {
  int yaw_pos = mav_trajectory_generation::derivative_order::POSITION;
  int yaw_vel = mav_trajectory_generation::derivative_order::VELOCITY;
  int yaw_acc = mav_trajectory_generation::derivative_order::ACCELERATION;
  double sample_time;
  std::vector<Eigen::VectorXd> yaw_vec, yawdot_vec, yawddot_vec;

  const double min_time = yaw_trajectory.getMinTime();
  const double max_time = yaw_trajectory.getMaxTime();

  yaw_trajectory.evaluateRange(min_time, max_time, dt, yaw_pos, 
                               &yaw_vec);
  yaw_trajectory.evaluateRange(min_time, max_time, dt, yaw_vel, 
                               &yawdot_vec);
  yaw_trajectory.evaluateRange(min_time, max_time, dt, yaw_acc, 
                               &yawddot_vec);

  for (int i = 0; i < yaw_vec.size(); i++){
    flatStates->PVAJS_array[i].yaw = yaw_vec[i][0];
    flatStates->PVAJS_array[i].yaw_dot = yawdot_vec[i][0];
    flatStates->PVAJS_array[i].yaw_ddot = yawddot_vec[i][0];
   }
}


double solveMinSnap(
	const mav_trajectory_generation::Vertex::Vector &vertices,
	const Eigen::VectorXd &segment_times,
	const int &dimension, 
	mav_trajectory_generation::Trajectory *trajectory){

  //Convert segment times to the appropriate type
  std::vector<double> stdSegment_times;
  eigenVectorXd2stdVector(segment_times, &stdSegment_times);

  //Solve optimization problem
  const int N = 10;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
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

double solveMinAcceleration(
  const mav_trajectory_generation::Vertex::Vector &vertices,
  const Eigen::VectorXd &segment_times,
  const int &dimension, 
  mav_trajectory_generation::Trajectory *trajectory){

  //Convert segment times to the appropriate type
  std::vector<double> stdSegment_times;
  eigenVectorXd2stdVector(segment_times, &stdSegment_times);

  return solveMinAcceleration(vertices, stdSegment_times, dimension, trajectory);
}

double solveMinAcceleration(
  const mav_trajectory_generation::Vertex::Vector &vertices,
  const std::vector<double> &segment_times,
  const int &dimension, 
  mav_trajectory_generation::Trajectory *trajectory){

  //Solve optimization problem
  const int N = 10;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
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

//Compute minimum snap trajectory with optimal segments and return minSnap cost
double solveMinSnapGradDescent(
  const mav_trajectory_generation::Vertex::Vector &vertices,
  const int &dimension, 
  Eigen::VectorXd &segment_times,
  mav_trajectory_generation::Trajectory *trajectory,
  Eigen::VectorXd *segment_times_out){

  //Get initial solution for minimum snap problem
  double cost;
  mav_trajectory_generation::Trajectory curTrajectory;
  cost = solveMinSnap(vertices, segment_times, dimension, &curTrajectory);

  //Declare variables for gradient descent
  const double FinalTime = curTrajectory.getMaxTime();
  const int m = segment_times.size();
  const double h = 0.00001;     //Small step for time gradient
  const double epsilon = 0.01;  //Condition for terminating gradient descent
  double step = std::numeric_limits<float>::infinity();
  double costNew;
  Eigen::VectorXd g = (-1.0/(m-1.0))*Eigen::MatrixXd::Ones(m,1);
  Eigen::VectorXd gradF = Eigen::MatrixXd::Zero(m,1);
  double init_cost = cost;
  Eigen::VectorXd best_segment_times;
  *trajectory = curTrajectory;

  //Gradient descent loop
  while(step > epsilon){

    //Calculate gradient
    Eigen::VectorXd g_i = g;
    Eigen::VectorXd segment_times_new = segment_times;
    for(int i = 0; i < m; i++){
      g_i = g;
      g_i[i] = 1;
      segment_times_new = segment_times + h*g_i;
      costNew = solveMinSnap(vertices, segment_times_new, dimension, &curTrajectory);
      gradF(i) = (costNew - cost)/h;
    }

    //Normalize gradient vector (its bigger value will be as big as the bigger segment time)
    gradF = segment_times_new.cwiseAbs().minCoeff()*gradF/gradF.cwiseAbs().maxCoeff();

    //Perform gradient descent
    double alpha = 0.9;           //Step size
    double curCost = std::numeric_limits<float>::infinity();
    double prevCost = std::numeric_limits<float>::infinity();
    best_segment_times = segment_times;
    double bestCost = cost;
    for (int j = 0; j < 6; j++){  //Here we only iterate 6 times before alpha becomes too small
      segment_times_new = segment_times - alpha*gradF;

      //Renormalize segment times to preserve final time
      segment_times_new = FinalTime*segment_times_new/segment_times_new.sum();

      curCost = solveMinSnap(vertices, segment_times_new, dimension, &curTrajectory);

      // if(curCost > prevCost) {
      //     break;
      // }

      if(curCost < bestCost){
        bestCost = curCost;
        best_segment_times = segment_times_new;
      }

      alpha = alpha*0.5;
      prevCost = curCost;
    }

    //Check if there was any improvement. Otherwise, stop iterations
    if(bestCost < cost){
      segment_times = best_segment_times;
      step = (cost - bestCost)/cost;
      cost = bestCost;
      *trajectory = curTrajectory;
    }
    else{
      break;
    }

  }
  ROS_INFO("improvement: %f", (init_cost-cost)/cost);

  // std::cout  << segment_times.transpose() << std::endl;

  *segment_times_out = best_segment_times;
  return cost;
}

void SolveNlopt(const std::vector<double> &segment_times, 
                const mav_trajectory_generation::Vertex::Vector &wp_vertices,
                const mav_trajectory_generation::Vertex::Vector &yaw_vertices,
                const double &dt, const uint &n_w,
                const double &max_vel, const double &max_acc, const double &max_jerk,
                mg_msgs::minSnapWpStamped::Response *res) {
  const int dimension = 3, yaw_dimension = 1;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  double cost = 0, yaw_cost = 0;

  //Get solution for minimum snap problem with optimal segment times
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  parameters.max_iterations = 1000;
  parameters.f_rel = 0.05;
  parameters.x_rel = 0.1;
  parameters.time_penalty = 0.1;
  parameters.initial_stepsize_rel = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;
  // parameters.use_soft_constraints = false;
  parameters.print_debug_info = false;
  const int N = 10;

  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters, false);
  opt.setupFromVertices(wp_vertices, segment_times, derivative_to_optimize);

  // Set inequality constraints
  if(max_vel > 0) {
    ROS_INFO("Maximum velocity: %f", max_vel);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_vel);
  }
  if(max_acc > 0) {
    ROS_INFO("Maximum acceleration: %f", max_acc);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_acc);
  }
  if(max_jerk > 0) {
    ROS_INFO("Maximum jerk: %f", max_jerk);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, max_jerk);
  }

  // Optimize
  opt.optimize();

  mav_trajectory_generation::OptimizationInfo optimization_info_ = opt.getOptimizationInfo();
  // std::cout << optimization_info_.cost_trajectory << std::endl;
  // std::cout << optimization_info_.cost_time << std::endl;
  

  mav_trajectory_generation::Segment::Vector segments;
  opt.getPolynomialOptimizationRef().getSegments(&segments);
  opt.getTrajectory(&trajectory);
  std::vector<double> stdSegment_timesFinal;
  opt.getPolynomialOptimizationRef().getSegmentTimes(&stdSegment_timesFinal);

  // for (uint i = 0; i < stdSegment_timesFinal.size(); i++) {
    // std::cout << "segment " << i << ": " << stdSegment_timesFinal[i] << std::endl;
  // }

  // Solve for yaw using the optimal segment times
  mav_trajectory_generation::Trajectory trajectory_yaw;
  yaw_cost = solveMinAcceleration(yaw_vertices, stdSegment_timesFinal, yaw_dimension,
                                  &trajectory_yaw);

  //Sample trajectory
  mg_msgs::PVAJS_array flatStates;
  std::vector<maxValues> maxVal;
  maxValues maxValTraj;
  trajectory2waypoint(trajectory, trajectory_yaw, dt, &flatStates, &maxVal, &maxValTraj);

  ROS_INFO("Max vel: %f\tMax acc:%f\tMax Jerk: %f", maxValTraj.max_vel, maxValTraj.max_acc, maxValTraj.max_jerk);

  std::cout << "final time: " << trajectory.getMaxTime() << std::endl;

  //Output
  res->flatStates = flatStates;
  res->cost.data = cost;

  std_msgs::Float32 dt_wp;
  for (int i = 0; i < n_w-1; i++){
    dt_wp.data = stdSegment_timesFinal[i];
    res->dt_out.push_back(dt_wp);
  }
}

void SolveNlopt(const std::vector<double> &segment_times, 
                const mav_trajectory_generation::Vertex::Vector &wp_vertices,
                const mav_trajectory_generation::Vertex::Vector &yaw_vertices,
                const double &dt, const uint &n_w,
                const double &max_vel, const double &max_acc, const double &max_jerk,
                mg_msgs::minSnapWpPVAJ::Response *res) {
  mg_msgs::minSnapWpStamped::Response res_local;
  SolveNlopt(segment_times, wp_vertices, yaw_vertices, dt,
             n_w, max_vel, max_acc, max_jerk, &res_local);
  res->flatStates = res_local.flatStates;
  res->dt_out = res_local.dt_out;
  res->cost = res_local.cost;
}