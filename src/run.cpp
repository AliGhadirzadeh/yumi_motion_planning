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

  string filename;
  param_node.getParam("/filenames/fixed_obstacles", filename);
  right_arm_planner.add_fixed_obstacles(filename);

  std::vector<double> right_joint_position;
  param_node.getParam("/initial_joint_position/right_arm", right_joint_position);
  std::vector<int> traj_indices;
  param_node.getParam("/trajectories/indices", traj_indices);
  string traj_filename;
  param_node.getParam("/filenames/trajs", traj_filename);

  int ntrajs;
  param_node.getParam("/trajectories/ntrajs", ntrajs);

  int execute_motion;
  param_node.getParam("/target/execute_motion", execute_motion);

  vector < vector <double> > states;
  //for(int i = 0; i < traj_indices.size(); i++)
  cout << ntrajs << endl;
  for(int i = 0; i < ntrajs; i++)
  {
    right_arm_planner.load_trajectory(traj_filename, i);
    vector<double> s;
    if (execute_motion)
    {
      vector <double> curr_rpy, curr_xyz;
      right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);
      sleep(3);
      //right_arm_planner.print_trajectory();
      right_arm_planner.execute_trajectory();
      curr_rpy = right_arm_planner.get_current_rpy();
      curr_xyz = right_arm_planner.get_current_xyz();
      cout << "actual pose:\t" << curr_xyz[0] << "\t" << curr_xyz[1] << "\t"<< curr_xyz[2] << endl;
      cout << "            \t" << curr_rpy[0] << "\t" << curr_rpy[1] << "\t"<< curr_rpy[2] << endl;
      s.push_back(curr_xyz[0]);
      s.push_back(curr_xyz[1]);
      s.push_back(curr_rpy[2]);
      sleep(2);
    }
    else
    {
      vector<double> last_joint_pose = right_arm_planner.get_trajectory_joint_position(-1);
      vector<double> pose = right_arm_planner.fk_solver(last_joint_pose);
      //cout << "pred pose:\t" << pose[0] << "\t" << pose[1] << "\t"<< pose[2] << endl;
      //cout << "            \t" << pose[3] << "\t" << pose[4] << "\t"<< pose[5] << endl;
      s.push_back(pose[0]);
      s.push_back(pose[1]);
      s.push_back(pose[5]);
    }
    states.push_back(s);

    //cout  << "state: " << s[0] << "\t" << s[1] << "\t" << s[2] << endl;
  }

  right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);
  save_data(traj_filename + "/states.txt", states);

  return 0;
}
