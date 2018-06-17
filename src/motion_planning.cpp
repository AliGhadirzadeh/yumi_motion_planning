#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

#define BASE_LINK_FRAME "yumi_base_link"
#define RIGHT_TOOL_TIP_FRAME "yumi_link_7_r"

void add_fixed_obstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                          string frame_id,
                          string fixed_obstacles_filename);

int main(int argc, char **argv) {
  // initialization
  ros::init(argc, argv, "motion_planning");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // moveit initialization
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup right_arm_group("right_arm");
  //moveit::planning_interface::MoveGroup left_arm_group("left_arm");

  right_arm_group.setPoseReferenceFrame(BASE_LINK_FRAME);
  right_arm_group.setEndEffectorLink(RIGHT_TOOL_TIP_FRAME);

  //left_arm_group.setPoseReferenceFrame(base_link_frame);
  //left_arm_group.setEndEffectorLink(left_link_frame);

  geometry_msgs::PoseStamped pos_right = right_arm_group.getCurrentPose(RIGHT_TOOL_TIP_FRAME);
  //geometry_msgs::PoseStamped pos_left = left_arm_group.getCurrentPose(left_link_frame);

  std::vector<double> joint_values;
  right_arm_group.getCurrentState()->copyJointGroupPositions(right_arm_group.getCurrentState()->getRobotModel()->getJointModelGroup(right_arm_group.getName()), joint_values);
  std::cout << "Right arm joint positions: ";
  for (int i = 0; i<7; i++)
    std::cout << joint_values[i] << " ";
  std::cout << std::endl;

  moveit::planning_interface::MoveGroup::Plan plan;

  //Right arm joint positions: 1.25485 -1.3206 0.255866 0.603699 4.68985 1.68474 -3.40205
  string obstacle_filename = "/home/algh/catkin_ws/src/yumi_motion_planning/files/fixed_obstacles.txt";
  add_fixed_obstacles(planning_scene_interface, right_arm_group.getPlanningFrame(), obstacle_filename);

  joint_values[0] = 1.2;
  joint_values[1] = -1.3;
  joint_values[2] = 0.25;
  joint_values[3] = 0.6;
  joint_values[4] = 4.0;
  joint_values[5] = 1.5;
  joint_values[6] = -3.0;

  right_arm_group.setJointValueTarget(joint_values);
  right_arm_group.plan(plan);
  right_arm_group.move();

  sleep(2.0);

  pos_right.pose.position.x = 0.416511;
  pos_right.pose.position.y = -0.0076971;
  pos_right.pose.position.z = 0.16559+0.02;
  pos_right.pose.orientation.x =0.721902;
  pos_right.pose.orientation.y =-0.691336;
  pos_right.pose.orientation.z =0.00563416;
  pos_right.pose.orientation.w =0.0296696;
  right_arm_group.setPoseTarget(pos_right);
  right_arm_group.plan(plan);
  right_arm_group.move();


  pos_right.pose.position.z = 0.16559;
  right_arm_group.setPoseTarget(pos_right);
  right_arm_group.plan(plan);
  right_arm_group.move();

  return 0;

  char user_input;
  while(1)
  {
    std::cin >> user_input;
    if(user_input == 'l')
      pos_right.pose.position.y += 0.01;
    else if(user_input == 'r')
      pos_right.pose.position.y -= 0.01;
    else if(user_input == 'u')
      pos_right.pose.position.z += 0.01;
    else if(user_input == 'd')
      pos_right.pose.position.z -= 0.01;
    else if(user_input == 'f')
      pos_right.pose.position.x += 0.01;
    else if(user_input == 'b')
      pos_right.pose.position.x -= 0.01;
    else if(user_input == 't')
      break;
    right_arm_group.setPoseTarget(pos_right);
    right_arm_group.plan(plan);
    right_arm_group.move();
    std::cout << "x: " << pos_right.pose.position.x << "\ty: " << pos_right.pose.position.y  << "\tz: " << pos_right.pose.position.z << std::endl;
    std::cout << "o.x: " << pos_right.pose.orientation.x << "\to.y: " << pos_right.pose.orientation.y  << "\to.z: " << pos_right.pose.orientation.z << "\to.w: " << pos_right.pose.orientation.w<<std::endl;
  }

/*
  pos_right.pose.position.z += 0.04;
  right_arm_group.setPoseTarget(pos_right);
  right_arm_group.plan(plan);
  right_arm_group.move();
  pos_right.pose.position.z -= 0.08;
  right_arm_group.setPoseTarget(pos_right);
  right_arm_group.plan(plan);
  right_arm_group.move();

*/

  std::cout << "everything is completed successfully." << std::endl;
  ros::shutdown();
  return 0;
}

void add_fixed_obstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                          string frame_id,
                          string fixed_obstacles_filename)
{
  ifstream file(fixed_obstacles_filename.c_str());
  if(file.is_open())
  {
    string line;
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
  {
    cout << "file could not be openned!" << endl;
  }
}
