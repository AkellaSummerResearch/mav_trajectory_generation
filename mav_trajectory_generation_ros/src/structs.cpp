#include <mav_trajectory_generation_ros/structs.h>

waypoint_and_trajectory::waypoint_and_trajectory(nav_msgs::Path Waypoints,
                                                 mav_trajectory_generation_ros::PVAJS_array flatStates) {
    Waypoints_ = Waypoints;
    flatStates_ = flatStates;
}