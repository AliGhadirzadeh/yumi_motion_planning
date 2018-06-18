#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import yumi_moveit_utils as yumi
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
import argparse

def run(planning_frame="/yumi_body"):
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """

    #Start by connecting to ROS and MoveIt!
    yumi.init_Moveit(planning_frame)

    # Print current joint angles
    yumi.print_current_joint_states(yumi.RIGHT)
    yumi.print_current_joint_states(yumi.LEFT)

    #sys.exit(0)

    # Reset YuMi joints to "home" position
    #yumi.reset_pose()

    # Drive YuMi end effectors to a desired position (pose_ee), and perform a grasping task with a given effort (grip_effort)
    # Gripper effort: opening if negative, closing if positive, static if zero
    pose_ee = [0.25, 0.15, 0.2, 0.0, 3.14, 0.0]
    pose_ee_t = [0.47185, -0.144102, 0.25, 0.0, 3.14, 0.0]
    pose_ee_t = [0.45, 0.0, 0.1525, 0, 3.14, 0.0]

    #yumi.open_grippers(yumi.LEFT)
    yumi.open_grippers(yumi.RIGHT)
    #yumi.move_global_planning(yumi.RIGHT, pose_ee_t)
    yumi.move_local_planning(yumi.RIGHT, pose_ee_t)
    rospy.sleep(2.0)

    #pose_ee_t[2] = 0.2
    #yumi.move_global_planning(yumi.RIGHT, pose_ee_t)
    #rospy.sleep(2.0)
    #yumi.close_grippers(yumi.RIGHT)
    #pose_ee_t[2] = 0.25
    #yumi.move_global_planning(yumi.RIGHT, pose_ee_t)
    #rospy.sleep(2.0)

    #pose_ee = [0.25, 0.15, 0.25, 0.0, 3.14, 0.0]

    #yumi.move_global_planning(yumi.LEFT, pose_ee)
    #rospy.sleep(2.0)
    #yumi.open_grippers(yumi.LEFT)

    #pose_ee = [0.2, -0.35, 0.25, 0.0, 3.14, 0.0]

    #yumi.move_global_planning(yumi.RIGHT, pose_ee)
    #yumi.open_grippers(yumi.RIGHT)
    #rospy.sleep(2.0)
    # Reset YuMi joints to "home" position

    #yumi.reset_pose()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--planning_frame",type=str, default="/yumi_base_link",
        help='Moveit planning frame')

    args = parser.parse_args(rospy.myargv()[1:])


    rospy.init_node('yumi_motion_planning_demo')

    try:
        run(args.planning_frame)

    	print "####################################     Program finished     ####################################"
    except rospy.ROSInterruptException:
        pass
