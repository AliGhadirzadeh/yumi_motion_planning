#ifndef MOVEIT_UTIL_H
#define MOVEIT_UTIL_H

#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;


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


// moveit_util class is defined here
class MoveItPlanner
{
private:
  moveit::planning_interface::MoveGroupInterface* move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
  string collision_object_frame_;
  int num_planning_tries_ = 20;

public:
  MoveItPlanner(std::string planning_group)
  {
    move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group);
    move_group_->setNumPlanningAttempts(25);
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
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    int n_tries = 0;
    double fraction = 0;
    double fraction_max = 0;
    while (fraction < 0.999 && n_tries < num_planning_tries_)
    {
      fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      if (fraction > fraction_max)
        fraction_max = fraction;
      n_tries++;
    }
    if(fraction_max < 0.999)
    {
      ROS_INFO("path not found - fraction:%f", fraction_max);
      return false;
    }
    else
    {
      ROS_INFO("Cartesian path successfully found - fraction:%f", fraction_max);
      move_group_->execute(plan_);
      //move_group_->move();
    }

    return true;
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
