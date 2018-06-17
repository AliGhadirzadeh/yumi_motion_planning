#include <yumi_motion_planning/moveit_util.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_trajectories");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle param_node;

  string run_mode = "all";
  if (argc > 1)
    run_mode = argv[1];

  if (run_mode != "all" &&  run_mode != "init")
  {
    ROS_ERROR("run_mode not defined!");
    return -1;
  }

  bool success;
  std::vector<double> right_joint_position;
  std::vector<double> left_joint_position;


  static const std::string planning_group_right = "right_arm";
  MoveItPlanner right_arm_planner(planning_group_right);


  /*
  // initialize the planner for both arms
  moveit::planning_interface::MoveGroupInterface move_group_right(planning_group_right);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group_right = move_group_right.getCurrentState()->getJointModelGroup(planning_group_right);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group_right.setNumPlanningAttempts(25);
  */

  // add all fixed obstacles
  string filename;
  param_node.getParam("/filenames/fixed_obstacles", filename);

  right_arm_planner.add_fixed_obstacles(filename);

  // move both of the arms to the initial position
  param_node.getParam("/initial_joint_position/right_arm", right_joint_position);
  right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);

  /*move_group_right.setJointValueTarget(right_joint_position);
  success = (move_group_right.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
    move_group_right.move();
  //TODO move the left arm as well
  */
  /*
  string collision_object_frame;
  add_fixed_obstacles(planning_scene_interface, filename, collision_object_frame);

  // initialization completed
  if(run_mode == "init")
    return 0;


  // move both of the arms to the initial position
  param_node.getParam("/initial_joint_position/right_arm", right_joint_position);
  move_group_right.setJointValueTarget(right_joint_position);
  success = (move_group_right.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
    move_group_right.move();
  //TODO move the left arm as well

  // add objects
  int n_objects = 0;
  param_node.getParam("/collision_objects/number", n_objects);

  for (int obj = 1; obj <= n_objects; obj++)
  {
    std::vector<float> object_dimension;
    param_node.getParam("/collision_objects/"+std::to_string(obj)+"/dimensions", object_dimension);
    std::vector<float> object_pose;
    param_node.getParam("/collision_objects/"+std::to_string(obj)+"/pose", object_pose);
    add_collision_object(planning_scene_interface, "object"+std::to_string(obj), object_dimension, create_pose(object_pose), collision_object_frame);
  }
  // plan trajectories to different target poses
  //double x_min, x_max, y_min, y_max, z_min, z_max;

  std::vector<float> target_pose;
  param_node.getParam("/target_position/right_arm", target_pose);
  geometry_msgs::Pose r_ef_pose = create_pose(target_pose);

  move_group_right.setPoseTarget(r_ef_pose);

  success = (move_group_right.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
    move_group_right.move();
  */
  /*
  sleep(1.0);
  moveit::core::RobotStatePtr current_state = move_group_right.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group_right, joint_group_positions);
  for(int i = 0; i < 7; i++)
    std::cout << joint_group_positions[i] << ",";
  std::cout << std::endl;
  */

  return 0;
}
