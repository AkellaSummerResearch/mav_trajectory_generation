#include "ros/ros.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "min_snap/minSnapStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <mav_trajectory_generation_ros/ros_visualization.h>


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
  ros::ServiceClient client = n.serviceClient<min_snap::minSnapStamped>("/minSnap");
  min_snap::minSnapStamped srv;

  //Populate waypoints
  geometry_msgs::PoseStamped Pos;
  int n_waypoints = 10;

  // for (int j = 0; j < 200; j = j + 100){
    // n_waypoints = 100 + j;

    nav_msgs::Path Waypoints;

    for(int i = 0; i < n_waypoints; i++){
      Eigen::Vector3d p = Eigen::Vector3d::Random();
      // Pos.pose.position = SetPoint(p(0), p(1), p(2));
      Pos.pose.position = SetPoint(pow(float(i)/3.0,2), float(i)/5.0, float(i)/10.0);
      Pos.header.stamp = ros::Time().fromSec(float(i));
      // Pos.pose.position = SetPoint(float(i), float(i), float(i));
      Waypoints.poses.push_back(Pos);
    }

    srv.request.input = Waypoints;
    if (client.call(srv))
    {
      ROS_INFO("Service returned succesfully! Publishing Markers...");
      
      // Publish into Rviz
      nav_msgs::Path Path_out = srv.response.output;
      mav_trajectory_generation::drawWaypoints(Waypoints, frame_id, &WaypointMarkers);
      mav_trajectory_generation::drawTrajectoryFromWaypoints(Path_out, frame_id, &TrajMarkers);

      // //Delete current markers
      pathMarker_pub.publish(deleteMarkers);
      wpMarker_pub.publish(deleteMarkers.markers[0]);
      ros::spinOnce();

      //Publish current markers
      int n_waypoints = WaypointMarkers.markers.size();
      pathMarker_pub.publish(TrajMarkers);
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

  

  ros::Rate loop_rate(1);

  while (ros::ok())
  {

    // Waypoint_pub.publish(Waypoints);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}