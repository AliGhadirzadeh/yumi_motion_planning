#include <yumi_motion_planning/moveit_util.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_trajectories");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle param_node;

  cout << "node started" << endl;
  srand (time(NULL));

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

  param_node.getParam("/target/area_lower_bound", goal_pose);  // fill goal_pose with min values

  std::cout << std::fixed << std::setprecision(2);
  std::vector<double> states(3);

  ros::Publisher r_gripper_pub;
  ros::NodeHandle r_gripper_node;
  r_gripper_pub = r_gripper_node.advertise<std_msgs::Float64>("/yumi/gripper_r_effort_cmd", 10);
  std_msgs::Float64 cmd;
  cmd.data = 10;
  r_gripper_pub.publish(cmd);



  int i = 0;
  clock_t begin, end;
  begin = clock();

  std::vector<double> init_pose = target_area_lowerbound;
  vector<geometry_msgs::Pose> waypoint_back;
  init_pose[2] += 0.1;
  init_pose[1] = -0.4;
  waypoint_back.push_back(create_pose(init_pose));

  for (double x = target_area_lowerbound[0]; x <= target_area_upperbound[0]; x += steps[0])
  {
    for (double y = target_area_lowerbound[1]; y <= target_area_upperbound[1]; y += steps[1])
    {
      for (double theta = target_area_lowerbound[5]; theta <= target_area_upperbound[5]; theta += steps[5])
      {
          // if (i < 32)
          // {
          //   i++;
          //   continue;
          // }
          double rx = target_area_lowerbound[0] + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(target_area_upperbound[0]-target_area_lowerbound[0])));
          double ry = target_area_lowerbound[1] + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(target_area_upperbound[1]-target_area_lowerbound[1])));
          double rtheta = target_area_lowerbound[5] + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(target_area_upperbound[5]-target_area_lowerbound[5])));

          for (int m=0; m<2; m++)
          {
            goal_pose = target_area_lowerbound;
            goal_pose[0] = rx;
            goal_pose[1] = ry;
            goal_pose[5] = rtheta;
            //random_goal = right_arm_planner.sample_random_goal_pose(target_area_lowerbound,target_area_upperbound);

            vector<geometry_msgs::Pose> waypoints, waypoints2;
            geometry_msgs::Pose p;
            goal_pose[2] += 0.07;
            p = create_pose(goal_pose);
            waypoints.push_back(p);
            waypoints2.push_back(p);
            p.position.z -= 0.072;
            waypoints.push_back(p);
            right_arm_planner.plan_cartesian_path(waypoints);

            cout << "target" << i << ":\t" << goal_pose[0] << "\t" << goal_pose[1] << "\t" << goal_pose[2] << endl;

            // // saving the states
            // states[0] = x;
            // states[1] = y;
            // states[2] = theta;
            // char state_filename[100];
            // sprintf(state_filename, "%s/states/%04d.txt", filename.c_str(), i);
            // save_data(state_filename, states);

            // saving the trajectory
            // right_arm_planner.save_trajectory(filename,i);
            //right_arm_planner.print_trajectory();
            if(execute_motion == 1)
            {
              right_arm_planner.execute_trajectory();
              if (m == 1)
              {
                // cout << "press any key to continue" << endl;
                // cin.get();
                sleep(2);
                cmd.data = 10;
              }
              else
              {
                sleep(2);
                cmd.data = -10;
              }
              r_gripper_pub.publish(cmd);
              sleep(2);
              vector <double> curr_rpy, curr_xyz;
              curr_rpy = right_arm_planner.get_current_rpy();
              curr_xyz = right_arm_planner.get_current_xyz();
              cout << "actual pose:\t" << curr_xyz[0] << "\t" << curr_xyz[1] << "\t"<< curr_xyz[2] << endl;
              cout << "            \t" << curr_rpy[0] << "\t" << curr_rpy[1] << "\t"<< curr_rpy[2] << endl;

              right_arm_planner.plan_cartesian_path(waypoints2);
              right_arm_planner.execute_trajectory();

              if (m == 0)
              {
                right_arm_planner.plan_cartesian_path(waypoint_back);
                right_arm_planner.execute_trajectory();
                // right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);

                end = clock();
                double diff = (double)(end - begin)/CLOCKS_PER_SEC;
                // cin.get();
                // sleep((0.2-diff)*100);
                cout << "press any key to continue " << i << " " << diff << endl;
                begin = clock();
              }

              sleep(2);
            }
          }

        i++;
        if (i >= max_ntraj)
        {
          cout << "max number of trajectories generated! terminating ... " << endl;
          x = target_area_upperbound[0] + 1;
          y = target_area_upperbound[1] + 1;
          theta = target_area_upperbound[5] + 1;
          cmd.data = 0;
          r_gripper_pub.publish(cmd);
          break;
        }
      }
    }
  }

  cout << "Program done successfully" << endl;

  return 0;
}
