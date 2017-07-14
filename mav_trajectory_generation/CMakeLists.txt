cmake_minimum_required(VERSION 2.8.3)
project(mav_trajectory_generation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_MODULE_PATH "/${CMAKE_SOURCE_DIR}/mav_trajectory_generation/cmake/modules;
                        ${CMAKE_MODULE_PATH}")

set(CMAKE_CXX_FLAGS "-std=c++11 -fopenmp -march=native -msse2 -DNDEBUG ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Cholmod REQUIRED)
find_package(SPQR REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CHOLMOD_INCLUDES}
  ${SPQR_INCLUDES}
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/motion_defines.cpp
  src/polynomial.cpp
  src/rpoly.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/vertex.cpp
)

############
# BINARIES #
############
cs_add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES})

#########
# TESTS #
#########
catkin_add_gtest(test_polynomial
  test/test_polynomial.cpp
)
target_link_libraries(test_polynomial ${PROJECT_NAME} ${catkin_LIBRARIES} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES})

catkin_add_gtest(test_polynomial_optimization
  test/test_polynomial_optimization.cpp
)
target_link_libraries(test_polynomial_optimization ${PROJECT_NAME} ${catkin_LIBRARIES} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
