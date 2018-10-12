#include "mg_msgs/PVAJS.h"
#include "mg_msgs/PVAJS_array.h"

#include <mutex>
#include <queue>

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
};

std::queue<waypoint_and_trajectory> wp_traj_list;
std::mutex m_wp_traj_list;
