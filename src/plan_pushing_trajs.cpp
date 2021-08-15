#include <yumi_motion_planning/moveit_util.h>
#include <std_msgs/Float64.h>
#include <time.h>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_pushing_trajs");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle param_node;

  cout << "plan_pushing_trajs node started" << endl;
  srand (time(NULL));

  cout << "the first random number: " << rand() << endl;

  int execute_motion;
  param_node.getParam("/target/execute_motion", execute_motion);


  string move_group_name;
  param_node.getParam("/moveit/move_group", move_group_name);
  MoveItPlanner right_arm_planner(move_group_name);

  // add fixed obstacles
  string filename;
  param_node.getParam("/filenames/fixed_obstacles", filename);
  right_arm_planner.add_fixed_obstacles(filename);

  std::vector<double> right_joint_position;
  param_node.getParam("/initial_joint_position/right_arm", right_joint_position);

  std::vector<double> target_area_lowerbound, target_area_upperbound;
  param_node.getParam("/target/area_lower_bound", target_area_lowerbound);
  param_node.getParam("/target/area_upper_bound", target_area_upperbound);

  int max_ntraj = 0;
  param_node.getParam("/target/max_number_of_traj", max_ntraj);
  int num_traj_per_position = 0;
  param_node.getParam("/target/num_traj_per_position", num_traj_per_position);

  param_node.getParam("/filenames/trajs", filename);
  char traj_filename[100];

  std::cout << std::fixed << std::setprecision(2);
  std::vector<double> states(9);
  std::vector<double> goal_pose(target_area_lowerbound);

  int traj_counter = 0;
  int reset_counter = 0;
  vector<geometry_msgs::Pose> waypoints;
  while (traj_counter < max_ntraj)
  {
    if (reset_counter <= 0)
    {
      // move the arms to the initial position
      right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);
      sleep(2);
      reset_counter = 5;
    }
    reset_counter--;

    // sample an initial position
    double x = target_area_lowerbound[0] + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(target_area_upperbound[0]-target_area_lowerbound[0])));
    double y = target_area_lowerbound[1] + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(target_area_upperbound[1]-target_area_lowerbound[1])));

    // move to the initial position
    goal_pose[0] = x;
    goal_pose[1] = y;
    waypoints.push_back(create_pose(goal_pose));
    right_arm_planner.plan_cartesian_path(waypoints);
    right_arm_planner.execute_trajectory();
    sleep(2);
    waypoints.clear();


    // sample multiple goal positions
    for (int samples = 0; samples < num_traj_per_position; samples++)
    {
      // measure the pose
      vector <double> curr_xyz;
      curr_xyz = right_arm_planner.get_current_xyz();
      states[0] = curr_xyz[0];
      states[1] = curr_xyz[1];
      vectory<double> joint_position = right_arm_planner.get_current_joints();
      for (int j = 0; j < 7; j++)
        states[j+2] = joint_position[j];

      // set the push goal
      double rand_num = static_cast<float>(rand())/(static_cast<float>(RAND_MAX));
      double dx = (rand_num - 0.5) *0.2;
      double dy = sqrt(0.01-dx*dx);
      if ((samples % 2) == 0)
    	  dy = -dy;
      goal_pose[0] = x + dx;
      goal_pose[1] = y + dy;
      waypoints.push_back(create_pose(goal_pose));
    	right_arm_planner.plan_cartesian_path(waypoints);
      waypoints.clear();
      // saving
      right_arm_planner.save_trajectory(filename, traj_counter);
      char state_filename[100];
      sprintf(state_filename, "%s/states/%04d.txt", filename.c_str(), traj_counter);
      save_data(state_filename, states);
      waypoints.clear();
      if (execute_motion)
      {
        right_arm_planner.execute_trajectory();
        sleep(2);
        // move back to the initial position
        goal_pose[0] = x;
        goal_pose[1] = y;
        geometry_msgs::Pose p;
        vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(create_pose(goal_pose));
        right_arm_planner.plan_cartesian_path(waypoints);
        right_arm_planner.execute_trajectory();
        waypoints.clear();
        sleep(2);
      }
      traj_counter++;
    	if (traj_counter >= max_ntraj)
              break;
    }
  }

  cout << "Program done successfully" << endl;
  return 0;
}
