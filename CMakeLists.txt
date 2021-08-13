cmake_minimum_required(VERSION 2.8.3)
project(yumi_motion_planning)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yumi_motion_planning
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface
  DEPENDS system_lib TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(yumi_motion_planning src/moveit_util.cpp)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(motion_planning src/motion_planning.cpp)
add_executable(plan_trajectories src/plan_trajectories.cpp)
add_executable(plan_pushing_trajs src/plan_pushing_trajs.cpp)
add_executable(run src/run.cpp)
#add_executable(add_obstacles src/add_obstacles.cpp)


#target_link_libraries(motion_planning ${catkin_LIBRARIES})
target_link_libraries(plan_trajectories ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(run ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(plan_pushing_trajs ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
#target_link_libraries(add_obstacles ${catkin_LIBRARIES})
