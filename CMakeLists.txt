cmake_minimum_required(VERSION 2.8.3)
project(yumi_motion_planning)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  kdl_wrapper
  cmake_modules
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(TinyXML REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yumi_motion_planning
  CATKIN_DEPENDS roscpp kdl_parser kdl_wrapper moveit_core moveit_ros_planning_interface
  DEPENDS system_lib TinyXML orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)


link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

add_library(yumi_motion_planning src/moveit_util.cpp)

add_executable(plan_trajectories src/plan_trajectories.cpp)
add_executable(plan_pushing_trajs src/plan_pushing_trajs.cpp)
add_executable(run src/run.cpp)
add_executable(run_pushes src/run_pushes.cpp)

target_link_libraries(plan_trajectories ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(run ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(plan_pushing_trajs ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(run_pushes ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
