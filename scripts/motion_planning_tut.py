#!/usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import moveit_utils as utils

def add_box(timeout, scene):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "yumi_base_link"
    box_pose.pose.position.x = 0.4
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.0
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    box_is_known=False
    box_is_attached=False

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)


# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
#print "============ Printing robot state"
#print robot.get_current_state()
#print ""


# plan to a joint target position
joint_goal = group.get_current_joint_values()
group.go(joint_goal, wait=True)
group.stop()

#print add_box(4, scene)

# Move to initial position
#x_p = 0.1
#y_p = -0.4
#z_p = 0.25
#roll_rad = 0.0
#pitch_rad = 3.14
#yaw_rad = 3.14/2
#pose_goal = utils.create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
#group.set_pose_target(pose_goal)
#plan = group.go(wait=True)
#group.stop()
#group.clear_pose_targets()

# Go on top of the object
x_p = 0.4
y_p = -0.1
z_p = 0.2
roll_rad = 0.0
pitch_rad = 3.14
yaw_rad = 0.0
pose_goal = utils.create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
group.set_pose_target(pose_goal)
plan = group.go(wait=True)
rospy.sleep(0.5)
group.stop()
group.clear_pose_targets()
rospy.sleep(1.0)

# move to grasp point
z_p = 0.16
pose_goal = utils.create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
group.set_pose_target(pose_goal)
plan = group.go(wait=True)
group.stop()
group.clear_pose_targets()

rospy.sleep(5)

# Back to initial position
group.go(joint_goal, wait=True)
group.stop()
