#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg


#Set the gripper to an effort value
def gripper_effort(gripper_id, effort):
    """Set gripper effort
    Sends an effort command to the selected gripper. Should be in the range of
    -20.0 (fully open) to 20.0 (fully closed)
    :param gripper_id: The ID of the selected gripper (LEFT or RIGHT)
    :param effort: The effort value for the gripper (-20.0 to 20.0)
    :type gripper_id: int
    :type effort: float
    :returns: Nothing
    :rtype: None
    """
    rospy.loginfo("Setting gripper " + str(gripper_id) + " to " + str(effort))
    rospy.loginfo('Setting gripper effort to ' + str(effort) + ' for arm ' + str(gripper_id))

    if (gripper_id == RIGHT):
        pubname = '/yumi/gripper_r_effort_cmd'

    if (gripper_id == LEFT):
        pubname = '/yumi/gripper_l_effort_cmd'

    pub = rospy.Publisher(pubname, std_msgs.msg.Float64, queue_size=10, latch=True)
    pub.publish(std_msgs.msg.Float64(effort))
rospy.sleep(1)
