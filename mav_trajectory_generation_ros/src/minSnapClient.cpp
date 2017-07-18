#include "ros/ros.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "mav_trajectory_generation_ros/minSnapStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include "mav_trajectory_generation_ros/PVAJS_array.h"


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
  
  //ROS initialization
  ros::init(argc, argv, "client");
  ros::NodeHandle n;

  //Publishers
  ros::Publisher pathMarker_pub = n.advertise<visualization_msgs::MarkerArray>("path_visualization_markers", 1000);
  ros::Publisher wpMarker_pub = n.advertise<visualization_msgs::Marker>("waypoint_visualization_markers", 1000);


  //visualization markers initialization
  visualization_msgs::MarkerArray TrajMarkers, WaypointMarkers, deleteMarkers;
  std::string frame_id = "fcu";
  mav_trajectory_generation::deleteMarkersTemplate(frame_id, &deleteMarkers);

  //Service client
  ros::ServiceClient client0 = n.serviceClient<mav_trajectory_generation_ros::minSnapStamped>("/minSnap");
  ros::ServiceClient client1 = n.serviceClient<mav_trajectory_generation_ros::minSnapStamped>("/minSnapNlopt");
  ros::ServiceClient client2 = n.serviceClient<mav_trajectory_generation_ros::minSnapStamped>("/minSnapOptTime");
  mav_trajectory_generation_ros::minSnapStamped srv;

  //Populate waypoints
  geometry_msgs::PoseStamped Pos;
  int n_waypoints = 10;

  ros::Rate loop_rate(1);

  // for (int j = 0; j < 100; j = j + 10){
  //   n_waypoints = 10 + j;

    nav_msgs::Path Waypoints;

    for(int i = 0; i < n_waypoints; i++){
      // Eigen::Vector3d p = Eigen::Vector3d::Random();
      // Pos.pose.position = SetPoint(p(0), p(1), p(2));
      // Pos.pose.position = SetPoint(pow(float(i)/10.0,2), float(i)/5.0, float(i)/10.0);
      Pos.pose.position = SetPoint(2.0*sin(float(i)/3.0), 2.0*cos(float(i)/3.0), float(i)/10.0);
      Pos.header.stamp = ros::Time().fromSec(double(i));
      // std::cout << Pos.header.stamp.sec << " " << Pos.header.stamp.nsec << std::endl;
      // Pos.pose.position = SetPoint(float(i), float(i), float(i));
      Waypoints.poses.push_back(Pos);
    }

    srv.request.Waypoints = Waypoints;
    if (client2.call(srv))
    {
      ROS_INFO("Service returned succesfully! Publishing Markers...");
      
      // Publish into Rviz
      int distance = 1.0;
      mav_msgs::EigenTrajectoryPoint::Vector states;
      mav_trajectory_generation::PVAJS_array2EigenTrajectoryPoint(srv.response.flatStates, &states);
      mav_trajectory_generation_ros::PVAJS_array flatStates = srv.response.flatStates;
      mav_trajectory_generation::drawWaypoints(Waypoints, frame_id, &WaypointMarkers);
      mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &TrajMarkers);
      // mav_trajectory_generation::drawTrajectoryFromWaypoints(Path_out, frame_id, &TrajMarkers);

      // //Delete current markers
      pathMarker_pub.publish(deleteMarkers);
      wpMarker_pub.publish(deleteMarkers.markers[0]);
      ros::spinOnce();

      loop_rate.sleep();

      //Publish current markers
      pathMarker_pub.publish(TrajMarkers);
      int n_waypoints = WaypointMarkers.markers.size();
      for (int j = 0; j < n_waypoints; j++){
          wpMarker_pub.publish(WaypointMarkers.markers[j]);
      }
      ros::spinOnce();
      
    }
    else
    {
      ROS_ERROR("Failed to call service...");
    }


  // }  

  

  

  while (ros::ok())
  {

    // Waypoint_pub.publish(Waypoints);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}