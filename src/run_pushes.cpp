#include <ros/ros.h>
#include <yumi_motion_planning/moveit_util.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <boost/lexical_cast.hpp>

using namespace std;

// global variables
vector<double> push_cmd;
double push_cmd_time = -1;
volatile sig_atomic_t flag = 0;
KDL::JntArray right_arm_joint_positions;
vector<double> right_arm_joint_velocity;

// function declarations
vector <double> str2vector(string str, int n_data);

void terminate(int sig){
  flag = 1;
}

void joint_state_callback(const sensor_msgs::JointState & msg)
{
    int right_arm_indecis[7] = {1,3,13,5,7,9,11};
    for(int i = 0;i < 7; i++)
    {
        right_arm_joint_positions(i) = msg.position[right_arm_indecis[i]];
        right_arm_joint_velocity[i] = msg.velocity[right_arm_indecis[i]];
    }
}

void update_traj_callback(const std_msgs::String & msg)
{
  push_cmd = str2vector(msg.data, 3);
  cout << push_cmd[0] << ' ' << push_cmd[1] << ' ' << push_cmd[2] << endl; 
  push_cmd_time = ros::Time::now().toSec();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "run_pushes_node");

  right_arm_joint_positions.resize(7);
  right_arm_joint_velocity.resize(7);
  push_cmd.resize(3);
  std_msgs::Float64 cmd;
  KDLWrapper right_arm_kdl_wrapper;
  KDL::Twist right_arm_cart_velocity;
  KDL::JntArray right_arm_joint_velcmd(7);
  string command_topic;

  signal(SIGINT, terminate);
  srand (time(NULL));
  std::cout << std::fixed << std::setprecision(2);

  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  ros::NodeHandle param_node;
  vector<ros::NodeHandle> r_velocity_command_node(7);
  vector<ros::Publisher> r_velocity_command_pub(7);
  int urdf_order[7] = {1,2,7,3,4,5,6};
  for(int i = 0; i < 7; i++)
  {
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_r/command";
    r_velocity_command_pub[i] = r_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
  }
  ros::NodeHandle cmd_node;
  ros::Subscriber cmd_subscriber = cmd_node.subscribe("/socket_string_msg", 10000, update_traj_callback);

  spinner.start();
  cout << "run_pushes_node started" << endl;

  // ros parameters
  string move_group_name;
  param_node.getParam("/moveit/move_group", move_group_name);
  MoveItPlanner right_arm_planner(move_group_name);
  double cmd_timeout = 0.5;
  param_node.getParam("/command_timeout", cmd_timeout);
  string filename;
  param_node.getParam("/filenames/fixed_obstacles", filename);
  right_arm_planner.add_fixed_obstacles(filename);
  std::vector<double> right_joint_position;
  param_node.getParam("/initial_joint_position/right_arm", right_joint_position);

  if(!right_arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
      ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
  right_arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);

  while(ros::ok())
  {
    if (flag)
      break;
    // move the arms to the initial position
    right_arm_planner.plan_and_move_to_joint_goal(right_joint_position);
    // wait until commands are received
    while(push_cmd_time < 0 && ~flag)
      usleep(1000);
    // follow the given commands
    while(ros::ok())
    {      
      double time_since_last_cmd = ros::Time::now().toSec() - push_cmd_time;
      if (time_since_last_cmd > cmd_timeout || flag)
      {
        cout << "Commands stopped - restarting!" << endl;
        cmd.data = 0;
        for(int i = 0; i < 7; i++)
          r_velocity_command_pub[i].publish(cmd);
        push_cmd_time = -1;
        break;
      }
      if (flag)
        break;
      right_arm_cart_velocity.vel = KDL::Vector(push_cmd[0], push_cmd[1], 0.0);
      right_arm_cart_velocity.rot = KDL::Vector(push_cmd[2], 0.0, 0.0);
      right_arm_kdl_wrapper.ik_solver_vel->CartToJnt(right_arm_joint_positions, right_arm_cart_velocity, right_arm_joint_velcmd);
      for(int i = 0; i < 7; i++)
      {
        cmd.data = right_arm_joint_velcmd(i);
        r_velocity_command_pub[i].publish(cmd);
      }
      usleep(100000); //wait 100 msec
    }
  }
  if (flag)
    cout << "Stop signal received! node is terminating ..." << endl;
  cout << "node terminated successfully" << endl;
  return 0;
}

vector <double> str2vector(string str, int n_data)
{
  vector<double> data;
  for (int j = 0; j < n_data; j++)
  {
    std::size_t p = str.find(" ");
    string value = str.substr(0, p);
    str = str.substr(p + 1);
    data.push_back(boost::lexical_cast<double>(value));
  }
  return data;
}
