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

geometry_msgs::Pose create_pose(std::vector<float> p)
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

public:
  MoveItPlanner(std::string planning_group)
  {
    move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group);
    move_group_->setNumPlanningAttempts(25);
  }
  ~MoveItPlanner()
  {

  }
  void plan_and_move_to_joint_goal(std::vector<double> joint_goal)
  {
    move_group_->setJointValueTarget(joint_goal);
    bool success = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
      move_group_->move();
  }


  void add_fixed_obstacles(string filename)
  {
    ifstream file(filename.c_str());
    if(file.is_open())
    {
      string line;
      getline(file, line);  // dummy read
      getline(file, line);
      string collision_object_frame = line;
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
        collision_object.header.frame_id = collision_object_frame;
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
};


void add_fixed_obstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                          string fixed_obstacles_filename,
                          string &collision_object_frame)
{
  ifstream file(fixed_obstacles_filename.c_str());
  if(file.is_open())
  {
    string line;
    getline(file, line);  // dummy read
    getline(file, line);
    string collision_object_frame = line;
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
      collision_object.header.frame_id = collision_object_frame;
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
    planning_scene_interface.addCollisionObjects(collision_objects);
  }
  else
    ROS_ERROR("add_obstacles: The obstacle file cannot be openned");
}

void add_collision_object(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                          string object_id,
                          std::vector<float> dimensions,
                          geometry_msgs::Pose pose,
                          string collision_object_frame)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = collision_object_frame;
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
  planning_scene_interface.addCollisionObjects(collision_objects);
}



#endif
