#ifndef MOVEIT_UTIL_H
#define MOVEIT_UTIL_H

#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tinyxml.h>
#include <boost/lexical_cast.hpp>

#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
using namespace std;

#define PI 3.1415926

geometry_msgs::Pose create_pose(double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::Pose pose;
  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q, pose.orientation);
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  return pose;
}
geometry_msgs::Pose create_pose(std::vector<double> p)
{
  geometry_msgs::Pose pose;
  double roll, pitch, yaw;
  pose.position.x = p[0];
  pose.position.y = p[1];
  pose.position.z = p[2];
  roll = p[3];
  pitch = p[4];
  yaw = p[5];
  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q, pose.orientation);
  return pose;
}
void save_data(string filename, vector<vector<double> > data)
{
  int nrows = data.size();
  int ncols = data[0].size();
  ofstream file(filename.c_str());
  if(file.is_open())
  {
    //file << nrows << "\n" << ncols << endl;
    for (int j = 0; j < nrows; j++)
    {
      for (int i = 0; i < (ncols-1); i++)
        file << data[j][i] << " ";
      file << data[j][(ncols-1)] << endl;
    }
    file.close();
  }
  else
    ROS_INFO("Could not open the file %s to write!", filename.c_str());
}
void save_data(string filename, vector<double>  data)
{
  int nrows = 1;
  int ncols = data.size();
  ofstream file(filename.c_str());
  if(file.is_open())
  {
    //file << nrows << "\n" << ncols << endl;
    for (int i = 0; i < (ncols-1); i++)
      file << data[i] << endl;
    file << data[ncols-1];
    file.close();
  }
  else
    ROS_INFO("Could not open the file %s to write!", filename.c_str());
}

vector < vector <double> > load_data(string filename)
{
  vector< vector<double> > data;
  vector <double> row;
  ifstream reader;
  reader.open(filename.c_str());
  string line;
  while (getline (reader,line) )
  {
    std::size_t p = line.find(" ");
    while(p != std::string::npos )
    {
      string value = line.substr(0, p);
      line = line.substr(p+1);
      p = line.find(" ");
      row.push_back(boost::lexical_cast<double>(value));
    }
    //cout << line << endl;
    //cout << boost::lexical_cast<double>(line) << endl;
    row.push_back(boost::lexical_cast<double>(line));
    data.push_back(row);
    row.clear();
  }
  reader.close();
  return data;
}

/*vector < vector <double> > load_data(string filename)
{
  vector<vector<double> >data;
  ifstream file(filename.c_str());
  int ncols, nrows;
  string line;
  if(file.is_open())
  {
    getline(file, line);
    nrows = atoi(line.c_str());
    getline(file, line);
    ncols = atoi(line.c_str());
    vector <double> rdata(ncols);
    for(int i = 0; i < nrows; i++)
    {
      for(int j =0; j < ncols; j++)
      {
        getline(file,line);
        rdata[j] = atof(line.c_str());
      }
      data.push_back(rdata);
    }
  }
  else
    ROS_INFO("Could not open the file %s to read!", filename.c_str());
  return data;
}*/


// moveit_util class is defined here
class MoveItPlanner
{
private:
  moveit::planning_interface::MoveGroupInterface* move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
  moveit_msgs::RobotTrajectory trajectory_;
  string collision_object_frame_;
  string planning_group_;

  // Forward kineamtic model
  robot_model_loader::RobotModelLoader* robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  // Other parameters
  int num_planning_tries_ = 20;
  int njoints_ = 7;

public:
  MoveItPlanner(std::string planning_group)
  {
    planning_group_ = planning_group;
    move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group);
    move_group_->setNumPlanningAttempts(25);

    robot_model_loader_ = new  robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model_ = robot_model_loader_->getModel();
    kinematic_state_ = robot_state::RobotStatePtr (new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group);
  }
  ~MoveItPlanner()
  {

  }


  bool plan_and_move_to_joint_goal(std::vector<double> joint_goal)
  {
    move_group_->setJointValueTarget(joint_goal);
    bool success = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
      move_group_->move();
    return success;
  }

  bool plan_and_move_to_pose_goal(std::vector<double> pose_goal)
  {
    geometry_msgs::Pose pose = create_pose(pose_goal);
    //geometry_msgs::Pose p = pose;
    //cout << p.position.x << "\t"<< p.position.y << "\t"<< p.position.z << "\t"<< p.orientation.x << "\t"<< p.orientation.y << "\t"<< p.orientation.z << "\t"<< p.orientation.w<< "\n";

    move_group_->setPoseTarget(pose);
    bool success = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
      move_group_->move();
    return success;
  }

  bool plan_cartesian_path(vector<geometry_msgs::Pose> waypoints)
  {
    move_group_->setMaxVelocityScalingFactor(0.1);
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    int n_tries = 0;
    double fraction = 0;
    double fraction_max = 0;
    while (fraction < 0.999 && n_tries < num_planning_tries_)
    {
      fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_);
      if (fraction > fraction_max)
        fraction_max = fraction;
      n_tries++;
    }
    if(fraction_max < 0.999)
    {
      ROS_INFO("path not found - fraction:%f", fraction_max);
      return false;
    }
    ROS_INFO("MoveItPlanner::plan_cartesian_path - Cartesian path successfully found - fraction: %f", fraction_max);
    return true;
  }

  bool execute_trajectory()
  {
    if(trajectory_.joint_trajectory.points.size() > 0)
    {
      plan_.trajectory_ = trajectory_;
      move_group_->execute(plan_);
      return true;
    }
    else
    {
      ROS_INFO("MoveItPlanner::execute_trajectory - Empty trajectory (failed)");
      return false;
    }
  }

  vector<double> get_trajectory_joint_position(int index)
  {
    vector<double> joint_position(njoints_);
    int npoints = trajectory_.joint_trajectory.points.size();
    assert(npoints > 0);
    assert (index < npoints);
    index = (index == -1) ? npoints-1 : index;
    assert (index > 0);

    for (int j = 0; j < njoints_; j++)
      joint_position[j] = trajectory_.joint_trajectory.points[index].positions[j];
    return joint_position;
  }

  vector<double> fk_solver(vector <double> joint_position)
  {
    std::vector<double> pose(6);

    kinematic_state_->setJointGroupPositions(joint_model_group_, joint_position);
    //ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    const Eigen::Affine3d& end_effector_state = kinematic_state_->getGlobalLinkTransform("yumi_link_7_r");

    Eigen::Vector3d yrp = end_effector_state.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d xyz = end_effector_state.translation();

    pose[0] = xyz(0); pose[1] = xyz(1); pose[2] = xyz(2);
    pose[3] = yrp(1); pose[4] = yrp(2); pose[5] = yrp(0);

    return pose;
  }


  void save_trajectory(string root, int traj_idx)
  {
    char filename[100];
    int npoints = trajectory_.joint_trajectory.points.size();

    vector <double> rdata(npoints);
    vector< vector <double> > data;

    sprintf(filename, "%s/time_steps/%04d.txt", root.c_str(), traj_idx);
    for (int i = 0; i < npoints; i++)
      rdata[i] = trajectory_.joint_trajectory.points[i].time_from_start.toSec();
    save_data(filename, rdata);
    for (int j = 0; j < njoints_; j++)
      data.push_back(rdata);
    sprintf(filename, "%s/positions/%04d.txt", root.c_str(), traj_idx);
    for (int j = 0; j < njoints_; j++)
      for (int i = 0; i < npoints; i++)
        data[j][i] = trajectory_.joint_trajectory.points[i].positions[j];
    save_data(filename, data);
    sprintf(filename, "%s/velocities/%04d.txt", root.c_str(), traj_idx);
    for (int j = 0; j < njoints_; j++)
      for (int i = 0; i < npoints; i++)
        data[j][i] = trajectory_.joint_trajectory.points[i].velocities[j];
    save_data(filename, data);
    sprintf(filename, "%s/accelerations/%04d.txt", root.c_str(), traj_idx);
    for (int j = 0; j < njoints_; j++)
      for (int i = 0; i < npoints; i++)
        data[j][i] = trajectory_.joint_trajectory.points[i].accelerations[j];
    save_data(filename, data);
  }
  void load_trajectory(string root, int traj_idx)
  {
    /*
    char filename[100];
    sprintf(filename, "%s/time_steps/%04d.txt", root.c_str(), traj_idx);
    vector<vector<double> > time_steps = load_data(filename);
    return;*/

    trajectory_.joint_trajectory.points.clear();
    char filename[100];
    sprintf(filename, "%s/positions/%04d.txt", root.c_str(), traj_idx);
    vector<vector<double> > pos = load_data(filename);
    sprintf(filename, "%s/velocities/%04d.txt", root.c_str(), traj_idx);
    vector<vector<double> > vel = load_data(filename);
    sprintf(filename, "%s/accelerations/%04d.txt", root.c_str(), traj_idx);
    vector<vector<double> > acc = load_data(filename);
    sprintf(filename, "%s/time_steps/%04d.txt", root.c_str(), traj_idx);
    vector<vector<double> > time_steps = load_data(filename);

    trajectory_.joint_trajectory.joint_names.clear();
    trajectory_.joint_trajectory.joint_names.resize(7);
    trajectory_.joint_trajectory.joint_names[0] = "yumi_joint_1_r";
    trajectory_.joint_trajectory.joint_names[1] = "yumi_joint_2_r";
    trajectory_.joint_trajectory.joint_names[2] = "yumi_joint_7_r";
    trajectory_.joint_trajectory.joint_names[3] = "yumi_joint_3_r";
    trajectory_.joint_trajectory.joint_names[4] = "yumi_joint_4_r";
    trajectory_.joint_trajectory.joint_names[5] = "yumi_joint_5_r";
    trajectory_.joint_trajectory.joint_names[6] = "yumi_joint_6_r";

    int npoints = pos[0].size();
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(7);
    point.velocities.resize(7);
    point.accelerations.resize(7);

    for (int p = 0; p < npoints; p++)
    {
      for(int j =0; j < njoints_; j++)
      {
        point.positions[j] = pos[j][p];
        point.velocities[j] = vel[j][p];
        point.accelerations[j] = acc[j][p];
      }
      ros::Duration t_from_start(time_steps[p][0]);
      point.time_from_start = t_from_start;

      trajectory_.joint_trajectory.points.push_back(point);
    }
    //cout << "Trajectory successfully loaded!" << endl;
  }

  void print_trajectory()
  {
    int npoints =trajectory_.joint_trajectory.points.size();

    cout <<"joints: " << endl;
    for (int i = 0; i < trajectory_.joint_trajectory.joint_names.size(); i++)
      cout << trajectory_.joint_trajectory.joint_names[i] << "\t";
    cout << endl;

    cout <<"npoints: " <<npoints << endl;
    cout << "time_steps: " << endl;
    for (int i = 0; i < npoints; i++)
      cout << trajectory_.joint_trajectory.points[i].time_from_start << "\t";
    cout << endl;

  }
  bool approach_from_top(double x, double y, double z, double angle)
  {
    bool success = true;
    vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose p;

    p = create_pose(x,y,z+0.05,0,angle,0);
    //cout << p.position.x << "\t"<< p.position.y << "\t"<< p.position.z << "\t"<< p.orientation.x << "\t"<< p.orientation.y << "\t"<< p.orientation.z << "\t"<< p.orientation.w<< "\n";
    waypoints.push_back(p);

    p.position.z -= 0.05;
    waypoints.push_back(p);

    plan_cartesian_path(waypoints);

    return success;
  }

  void shift_end_effector(double delta_x, double delta_y, double delta_z)
  {
    vector <double> pose = get_current_xyz();
    vector <double> rpy = get_current_rpy();
    pose.push_back(rpy[0]);
    pose.push_back(rpy[1]);
    pose.push_back(rpy[2]);
    pose[0] += delta_x;
    pose[1] += delta_y;
    pose[2] += delta_z;
    plan_and_move_to_pose_goal(pose);
  }

  vector<double> sample_random_goal_pose(vector<double> lower_bound, vector<double> upper_bound)
  {
    int d = lower_bound.size();
    vector<double> goal_pose(d);
    for(int i = 0; i < d; i++)
    {
      double r = ((double) rand() / (RAND_MAX));
      goal_pose[i] = (upper_bound[i]-lower_bound[i]) * r + lower_bound[i];
    }
    return goal_pose;
  }

  geometry_msgs::Pose get_current_pose()
  {
    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped pose_stmp =  move_group_->getCurrentPose ();
    pose = pose_stmp.pose;
    return pose;
  }

  vector <double> get_current_rpy()
  {
    std::vector<double> rpy = move_group_->	getCurrentRPY();
    for(int i = 0; i < 3; i++)
    {
      while(rpy[i] > PI)
        rpy[i] -= PI;
      while(rpy[i] < 0)
        rpy[i] += PI;
    }
    return rpy;
  }

  vector <double> get_current_xyz()
  {
    vector<double> position(3);
    geometry_msgs::PoseStamped pose_stmp =  move_group_->getCurrentPose();
    position[0] = pose_stmp.pose.position.x;
    position[1] = pose_stmp.pose.position.y;
    position[2] = pose_stmp.pose.position.z;
    return position;
  }

  vector<double> get_current_joints()
  {
    const robot_state::JointModelGroup* joint_model_group = move_group_->getCurrentState()->getJointModelGroup(planning_group_);
    vector<double> joint_position(njoints_);
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_position);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    return joint_position;
  }

  void add_fixed_obstacles(string filename)
  {
    ifstream file(filename.c_str());
    if(file.is_open())
    {
      string line;
      getline(file, line);  // dummy read
      getline(file, line);
      collision_object_frame_ = line;
      getline(file, line);  // dummy read
      getline(file, line);
      int n_obstacles = stoi(line);
      getline(file, line);  // dummy read
      std::vector<moveit_msgs::CollisionObject> collision_objects;
      double roll, pitch, yaw;
      double x, y, z;
      for (int i = 0; i < n_obstacles; i++)
      {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = collision_object_frame_;
        getline(file, line);
        collision_object.id = line.c_str();
        shape_msgs::SolidPrimitive primitive;
        geometry_msgs::Pose obj_pose;
        getline(file, line);
        if(strcmp(line.c_str(), "box") == 0)
        {
          primitive.type = primitive.BOX;
          // dimensions
          primitive.dimensions.resize(3);
          getline(file, line);
          primitive.dimensions[0] = stof(line);
          getline(file, line);
          primitive.dimensions[1] = stof(line);
          getline(file, line);
          primitive.dimensions[2] = stof(line);
          // position
          getline(file, line);
          x = stof(line);
          getline(file, line);
          y = stof(line);
          getline(file, line);
          z = stof(line);
          // orientation
          getline(file, line);
          roll = stof(line);
          getline(file, line);
          pitch = stof(line);
          getline(file, line);
          yaw = stof(line);
          obj_pose = create_pose(x,y,z, roll, pitch, yaw);
        }
        // TODO: complete the rest of shapes
        /*
        else if(strcmp(line, "sphere") == 0)
          primitive.type = primitive.SPHERE;
        else if(strcmp(line, "cylinder") == 0)
          primitive.type = primitive.CYLINDER;
        else if(strcmp(line, "cone") == 0)
          primitive.type = primitive.CONE;
        */
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(obj_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
      }
      planning_scene_interface_.addCollisionObjects(collision_objects);
    }
    else
      ROS_ERROR("add_obstacles: The obstacle file cannot be openned");
  }
  void add_collision_object(string object_id,
                            std::vector<float> dimensions,
                            geometry_msgs::Pose pose)
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = collision_object_frame_;
    collision_object.id = object_id;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dimensions[0];
    primitive.dimensions[1] = dimensions[1];
    primitive.dimensions[2] = dimensions[2];


    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface_.addCollisionObjects(collision_objects);
  }
};

#endif
