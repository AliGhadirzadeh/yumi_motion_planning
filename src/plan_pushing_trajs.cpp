#include <yumi_motion_planning/moveit_util.h>
#include <std_msgs/Float64.h>
#include <time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_pushing_trajs");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle param_node;

  cout << "plan_pushing_trajs node started" << endl;

  int execute_motion;
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
  // right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);

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

  std::vector<double> target_area_lowerbound, target_area_upperbound, goal_pose;
  param_node.getParam("/target/area_lower_bound", target_area_lowerbound);
  param_node.getParam("/target/area_upper_bound", target_area_upperbound);
  std::vector<double> steps;
  param_node.getParam("/target/steps", steps);

  int max_ntraj = 0;
  param_node.getParam("/target/max_number_of_traj", max_ntraj);

  param_node.getParam("/filenames/trajs", filename);
  char traj_filename[100];

  std::cout << std::fixed << std::setprecision(2);
  std::vector<double> states(3);

  ros::Publisher r_gripper_pub;
  ros::NodeHandle r_gripper_node;
  r_gripper_pub = r_gripper_node.advertise<std_msgs::Float64>("/yumi/gripper_r_effort_cmd", 10);
  std_msgs::Float64 cmd;
  cmd.data = 10;
  r_gripper_pub.publish(cmd);
  int traj_counter = 0;

  for (double x = target_area_lowerbound[0]; x <= target_area_upperbound[0]; x += steps[0])
  {
    for (double y = target_area_lowerbound[1]; y <= target_area_upperbound[1]; y += steps[1])
    {
      // get current position
      vector <double> curr_rpy, curr_xyz;
      curr_rpy = right_arm_planner.get_current_rpy();
      curr_xyz = right_arm_planner.get_current_xyz();
      cout << "actual pose:\t" << curr_xyz[0] << "\t" << curr_xyz[1] << "\t"<< curr_xyz[2] << endl;
      cout << "            \t" << curr_rpy[0] << "\t" << curr_rpy[1] << "\t"<< curr_rpy[2] << endl;
      // setting a new goal position
      geometry_msgs::Pose p;
      vector<geometry_msgs::Pose> waypoints;
      goal_pose = target_area_lowerbound;
      goal_pose[0] = curr_xyz[0];
      goal_pose[1] = curr_xyz[1];
      goal_pose[2] = curr_xyz[2];
      goal_pose[3] = curr_rpy[0];
      goal_pose[4] = curr_rpy[1];
      goal_pose[5] = curr_rpy[2];
      goal_pose[1] += 0.02;
      p = create_pose(goal_pose);
      waypoints.push_back(p);
      right_arm_planner.plan_cartesian_path(waypoints);
      right_arm_planner.execute_trajectory();
      traj_counter++;
      if (traj_counter >= max_ntraj)
        break;
    }
    if (traj_counter >= max_ntraj)
    {
      cout << "max number of trajectories generated! terminating ... " << endl;
      break;
    }
  }
  // termination
  cmd.data = 0;
  r_gripper_pub.publish(cmd);

  cout << "Program done successfully" << endl;
  return 0;
}
