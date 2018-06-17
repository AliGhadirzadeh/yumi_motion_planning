#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

void add_fixed_obstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                          string fixed_obstacles_filename);

int main(int argc, char **argv) {
  // initialization
  ros::init(argc, argv, "add_fixed_obstacles");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  string obstacle_filename;
  if(argc > 1)
    obstacle_filename = argv[1];
  else
  {
    ROS_ERROR("add_obstacles: The obstacle filename is not specified");
    return -1;
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  add_fixed_obstacles(planning_scene_interface, obstacle_filename);

  ros::shutdown();
  return 0;
}

void add_fixed_obstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                          string fixed_obstacles_filename)
{
  ifstream file(fixed_obstacles_filename.c_str());
  if(file.is_open())
  {
    string line;
    getline(file, line);
    string frame_id = line;
    getline(file, line);
    int n_obstacles = stoi(line);
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    for (int i = 0; i < n_obstacles; i++)
    {
      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = frame_id;
      getline(file, line);
      collision_object.id = line.c_str();
      shape_msgs::SolidPrimitive primitive;
      geometry_msgs::Pose obj_pose;
      getline(file, line);
      if(strcmp(line.c_str(), "box") == 0)
      {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        getline(file, line);
        primitive.dimensions[0] = stof(line);
        getline(file, line);
        primitive.dimensions[1] = stof(line);
        getline(file, line);
        primitive.dimensions[2] = stof(line);
        getline(file, line);
        obj_pose.orientation.w = stof(line);
        getline(file, line);
        obj_pose.position.x = stof(line);
        getline(file, line);
        obj_pose.position.y = stof(line);
        getline(file, line);
        obj_pose.position.z = stof(line);
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
