cmake_minimum_required(VERSION 2.8.3)
project(mav_trajectory_generation_ros)

set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_MODULE_PATH "/${CMAKE_SOURCE_DIR}/mav_trajectory_generation/cmake/modules;
                        ${CMAKE_MODULE_PATH}")

set(CMAKE_CXX_FLAGS "-std=c++11 -fopenmp -march=native -msse2 -DNDEBUG ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin_simple REQUIRED
  cmake_modules
  geometry_msgs
  nav_msgs
  message_generation
  roscpp
  std_msgs
)

find_package(Cholmod REQUIRED)
find_package(SPQR REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CHOLMOD_INCLUDES}
  #${SPQR_INCLUDES}
)

catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/feasibility_analytic.cpp
  src/feasibility_base.cpp
  src/feasibility_recursive.cpp
  src/feasibility_sampling.cpp
  src/input_constraints.cpp
  src/ros_conversions.cpp
  src/ros_visualization.cpp
  src/trajectory_sampling.cpp
)

#Create service types
add_service_files( FILES
  minSnapStamped.srv
)

#Create message types
add_message_files(DIRECTORY msg FILES
   PVAJS.msg
   PVAJS_array.msg
 )

generate_messages( DEPENDENCIES
  nav_msgs
  std_msgs
  geometry_msgs
)

############
# BINARIES #
############
cs_add_executable(trajectory_sampler_node
  src/trajectory_sampler_node.cpp
  src/HelperFunctions/helper.cpp
)
target_link_libraries(trajectory_sampler_node ${PROJECT_NAME} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES})

add_executable(minSnap_Node src/minSnapNode.cpp
                            src/trajectory_sampling.cpp
                            src/ros_visualization.cpp
                            src/ros_conversions.cpp
                            src/minSnapFunctions/minSnapFcns.cpp
                            src/HelperFunctions/helper.cpp
                            src/HelperFunctions/QuatRotEuler.cpp)
add_dependencies(minSnap_Node msg_pkg_generate_messages_cpp)
target_link_libraries(minSnap_Node ${catkin_LIBRARIES} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES})

add_executable(client_Node src/minSnapClient.cpp
                           src/trajectory_sampling.cpp
                           src/ros_visualization.cpp
                           src/ros_conversions.cpp
                           src/HelperFunctions/helper.cpp
                           src/HelperFunctions/QuatRotEuler.cpp)
add_dependencies(client_Node msg_pkg_generate_messages_cpp)
target_link_libraries(client_Node ${catkin_LIBRARIES} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES})



#########
# TESTS #
#########
catkin_add_gtest(test_feasibility
  test/test_feasibility.cpp
)
target_link_libraries(test_feasibility ${PROJECT_NAME} ${catkin_LIBRARIES})



##########
# EXPORT #
##########
cs_install()
cs_export()
