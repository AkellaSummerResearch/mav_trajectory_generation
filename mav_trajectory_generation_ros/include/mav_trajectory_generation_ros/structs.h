#ifndef STRUCTS_H
#define STRUCTS_H

#include "nav_msgs/Path.h"
#include "mg_msgs/PVAJS.h"
#include "mg_msgs/PVAJS_array.h"
#include "mg_msgs/PVAJ_request.h"

#include <mutex>
#include <queue>

struct maxValues {
  double max_vel = 0;
  double max_acc = 0;
  double max_jerk = 0;
  double max_snap = 0;
};

class waypoint_and_trajectory {
 public:
  nav_msgs::Path Waypoints_;
  mg_msgs::PVAJS_array flatStates_;

  waypoint_and_trajectory() {}
  waypoint_and_trajectory(nav_msgs::Path Waypoints,
                          mg_msgs::PVAJS_array flatStates) {
    Waypoints_ = Waypoints;
    flatStates_ = flatStates;
  }
  waypoint_and_trajectory(std::vector<mg_msgs::PVAJ_request> PVAJ_array,
                          mg_msgs::PVAJS_array flatStates) {
  	for (uint i = 0; i < PVAJ_array.size(); i++) {
  		geometry_msgs::PoseStamped pose;
  		pose.pose.position = PVAJ_array[i].Pos;
  		pose.pose.orientation.w  = cos(PVAJ_array[i].yaw/2.0);
  		pose.pose.orientation.z  = sin(PVAJ_array[i].yaw/2.0);
  		Waypoints_.poses.push_back(pose);
  	}
    flatStates_ = flatStates;
  }
};

#endif  // STRUCTS_H