cmake_minimum_required(VERSION 2.8.3)
project(gbplanner)
# Set OCTOMAP or VOXBLOX
option(USE_OCTOMAP "Use OCTOMAP or VOXBLOX" 0)
if (USE_OCTOMAP)
  add_definitions(-DUSE_OCTOMAP=${USE_OCTOMAP})
endif (USE_OCTOMAP)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)


catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DECOMP_UTIL_INCLUDE_DIRS})

if (USE_OCTOMAP)
cs_add_library(
  ${PROJECT_NAME}
  src/graph.cpp
  src/graph_manager.cpp
  src/gbplanner.cpp
  src/rrg.cpp
  src/trajectory.cpp
  src/params.cpp
  src/random_sampler.cpp
  src/gbplanner_rviz.cpp
  src/map_manager_octomap_impl.cpp
  src/geofence_manager.cpp
  src/PUTN_planner.cpp
  src/PUTN_classes.cpp
  src/PUTN_vis.cpp)
else (USE_OCTOMAP)
cs_add_library(
  ${PROJECT_NAME}
  src/graph.cpp
  src/graph_manager.cpp
  src/gbplanner.cpp
  src/rrg.cpp
  src/trajectory.cpp
  src/params.cpp
  src/random_sampler.cpp
  src/gbplanner_rviz.cpp
  src/map_manager_voxblox_impl.cpp
  src/geofence_manager.cpp
  src/PUTN_planner.cpp
  src/PUTN_classes.cpp
  src/PUTN_vis.cpp)
endif (USE_OCTOMAP)

cs_add_executable(gbplanner_node src/gbplanner_ros_node.cpp)
target_link_libraries(gbplanner_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)
