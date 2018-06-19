#include <yumi_motion_planning/moveit_util.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_trajectories");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle param_node;

  bool success;
  std::vector<double> right_joint_position;
  std::vector<double> left_joint_position;

  MoveItPlanner right_arm_planner("right_arm");

  // add all fixed obstacles
  string filename;
  param_node.getParam("/filenames/fixed_obstacles", filename);
  right_arm_planner.add_fixed_obstacles(filename);

  // move both of the arms to the initial position
  param_node.getParam("/initial_joint_position/right_arm", right_joint_position);
  right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);

  // add objects
  int n_objects = 0;
  param_node.getParam("/collision_objects/number", n_objects);
  for (int obj = 1; obj <= n_objects; obj++)
  {
    std::vector<float> object_dimension;
    param_node.getParam("/collision_objects/"+std::to_string(obj)+"/dimensions", object_dimension);
    std::vector<double> object_pose;
    param_node.getParam("/collision_objects/"+std::to_string(obj)+"/pose", object_pose);
    right_arm_planner.add_collision_object("object"+std::to_string(obj), object_dimension, create_pose(object_pose));
  }

  //std::vector<double> pose_goal;
  //param_node.getParam("/target_position/right_arm", pose_goal);
  //right_arm_planner.plan_and_move_to_pose_goal(pose_goal);
  //right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);

  right_arm_planner.approach_from_top(0.45, 0.0, 0.1525, 3.14);
  right_arm_planner.save_trajectory("/home/algh/catkin_ws/src/yumi_motion_planning/files/1.txt");
  ROS_INFO("Program done successfully");
  

  return 0;
}
