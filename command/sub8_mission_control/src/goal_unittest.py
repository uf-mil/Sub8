#! /usr/bin/env python

import rospy
import actionlib
import roslib; roslib.load_manifest('sub8_mission_control')
from goal_client import *  
from nav_msgs.msg import Odometry

if __name__ == '__main__':

    """ Run unittest on a functions in the goal system
        TODO: add relative and absolute moves to testing file
    """

    rospy.init_node('goal_client_unittest')
    sub = goal_proxy(testing = True)

    to_send = Odometry()
    to_send.pose.pose.position.z -= 10
    test = sub.move_down(10)
    assert test == to_send
    rospy.loginfo("Test passed")

    to_send = Odometry()
    to_send.pose.pose.position.z += 10
    test = sub.move_up(10)
    assert test == to_send
    rospy.loginfo("Test passed")

    to_send = Odometry()
    to_send.pose.pose.position.y -= 10
    test = sub.move_left(10)
    assert test == to_send
    rospy.loginfo("Test passed")

    to_send = Odometry()
    to_send.pose.pose.position.y += 10
    test = sub.move_right(10)
    assert test == to_send
    rospy.loginfo("Test passed")

    to_send = Odometry()
    to_send.pose.pose.position.x += 10
    test = sub.move_forward(10)
    assert test == to_send
    rospy.loginfo("Test passed")

    to_send = Odometry()
    to_send.pose.pose.position.x -= 10
    test = sub.move_back(10)
    assert test == to_send
    rospy.loginfo("Test passed")

    rospy.loginfo("All tests passed")



