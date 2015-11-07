#!/usr/bin/env python

import unittest
import numpy as np
import rospy
from sub8_msgs.msg import Waypoint
from sub8_mission_control import goal_proxy

class TestROSTools(unittest.TestCase):

    def test_down(self):
        sub = goal_proxy(testing = True)
        to_send = Waypoint()
        to_send.pose.position.z -= 10
        test = sub.move_down(10)
        assert test == to_send
        rospy.loginfo("Test passed")

    def test_up(self):
        sub = goal_proxy(testing = True)
        to_send = Waypoint()
        to_send.pose.position.z += 10
        test = sub.move_up(10)
        assert test == to_send
        rospy.loginfo("Test passed")

    def test_left(self):
        sub = goal_proxy(testing = True)
        to_send = Waypoint()
        to_send.pose.position.y -= 10
        test = sub.move_left(10)
        assert test == to_send
        rospy.loginfo("Test passed")

    def test_right(self):
        sub = goal_proxy(testing = True)
        to_send = Waypoint()
        to_send.pose.position.y += 10
        test = sub.move_right(10)
        assert test == to_send
        rospy.loginfo("Test passed")

    def test_forward(self):
        sub = goal_proxy(testing = True)
        to_send = Waypoint()
        to_send.pose.position.x += 10
        test = sub.move_forward(10)
        assert test == to_send
        rospy.loginfo("Test passed")

    def test_back(self):
        sub = goal_proxy(testing = True)
        to_send = Waypoint()
        to_send.pose.position.x -= 10
        test = sub.move_back(10)
        assert test == to_send
        rospy.loginfo("Test passed")


if __name__ == '__main__':
    rospy.init_node("goal_client_test", anonymous=True)
    suite = unittest.TestLoader().loadTestsFromTestCase(TestROSTools)
    unittest.TextTestRunner(verbosity=2).run(suite)
