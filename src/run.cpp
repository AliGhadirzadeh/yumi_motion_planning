#include <yumi_motion_planning/moveit_util.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "run_traj");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle param_node;

  string move_group_name;
  param_node.getParam("/moveit/move_group", move_group_name);
  MoveItPlanner right_arm_planner(move_group_name);
  string traj_filename;
  param_node.getParam("/filenames/trajs", traj_filename);
  right_arm_planner.load_trajectory(traj_filename);

/*  int execute_motion;
  param_node.getParam("/target/execute_motion", execute_motion);

  std::vector<double> right_joint_position;
  std::vector<double> left_joint_position;

  string move_group_name;
  param_node.getParam("/moveit/move_group", move_group_name);
  MoveItPlanner right_arm_planner(move_group_name);

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

  std::vector<double> target_area_lowerbound, target_area_upperbound, random_goal;
  param_node.getParam("/target/area_lower_bound", target_area_lowerbound);
  param_node.getParam("/target/area_upper_bound", target_area_upperbound);
  int n_traj = 0;
  param_node.getParam("/target/number_of_traj", n_traj);

  param_node.getParam("/filenames/trajs", filename);
  char traj_filename[100];
  for(int i = 0; i < n_traj; i++ )
  {
    random_goal = right_arm_planner.sample_random_goal_pose(target_area_lowerbound,target_area_upperbound);
    vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose p;
    random_goal[2] += 0.05;
    p = create_pose(random_goal);
    waypoints.push_back(p);
    p.position.z -= 0.05;
    waypoints.push_back(p);
    right_arm_planner.plan_cartesian_path(waypoints);
    sprintf(traj_filename, "%s/%04d.txt", filename.c_str(), i);
    cout << traj_filename << endl;
    right_arm_planner.save_trajectory(traj_filename);
    if(execute_motion == 1)
    {
      right_arm_planner.execute_trajectory();
      sleep(2);
      right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);
      sleep(2);
    }
  }
  ROS_INFO("Program done successfully");*/

  return 0;
}
