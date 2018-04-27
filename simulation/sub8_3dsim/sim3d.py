#!/usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import tf.transformations as trns
from mil_tools import numpy_to_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, Wrench, Pose, Point, Quaternion, Twist, Vector3
from mil_ros_tools import rosmsg_to_numpy

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

class Subsim():
    '''
    A simple 2D simulation of the kinematics of NaviGator.
    '''
    def __init__(self):
        # Used to publish current state
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=1)

        # Set initial state from constructor
        self.pose = Pose(position=Point(0,0,0), orientation=Quaternion(0,0,0,1))
        self.twist = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
        self.wrench = Wrench(force=Vector3(0,0,0), torque=Vector3(0,0,0))

        # Get other contants from ROS params
        self.get_params()

        # Subscribe
        rospy.Subscriber('/wrench_actual', WrenchStamped, self.actual_wrench_cb)

        # Start timer to run simulator
        rospy.Timer(rospy.Duration(self.update_period), self.timer_cb)

    def get_params(self):
        '''
        Load important configurable constants from ROS params
        '''
        mass = rospy.get_param('~mass')
        rotational_inertia = rospy.get_param('~rotational_inertia')
        self.inertia = np.float64([mass, mass, rotational_inertia])
        self.drag = np.float64(rospy.get_param('~drag'))
        self.update_period = rospy.get_param('~update_period', 0.1)
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.body_frame = rospy.get_param('~body_frame', 'base_link')

    def actual_wrench_cb(self, msg):
        self.wrench = msg.wrench

    def timer_cb(self, timer_event):
        '''
        Each time timer triggers, update state and publish odometry
        '''
        self.step(self.update_period, self.wrench)
        self.publish_odom()

    def step(self, dt, wrench):
        '''
        Simulate new pose and twist given a time delta and a force/torque applied to NaviGator
        '''
        position = rosmsg_to_numpy(self.pose.position)
        twist_linear = rosmsg_to_numpy(self.twist.linear)
        position = position + twist_linear + 0.5 * rosmsg_to_numpy(wrench.force) * dt**2
        self.pose.position = Point(*position)

        orientation = rosmsg_to_numpy(self.pose.orientation)

        twist_angular = rosmsg_to_numpy(self.twist.angular)
        twist_angular_as_quat = np.hstack((np.array([0]), twist_angular))

        wrench_torque_as_quat = np.hstack((np.array([0]), rosmsg_to_numpy(wrench.torque)))

        orientation = orientation + 0.5 * quaternion_multiply(orientation, wrench_torque_as_quat) 

        orientation = orientation / np.linalg.norm(orientation)
        self.pose.orientation = Quaternion(*orientation)

    def publish_odom(self):
        '''
        Publish to odometry with latest pose and twist
        '''
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose = self.pose
        self.odom_publisher.publish(odom)

if __name__ == '__main__':
    rospy.init_node('sub8_3Dsim')
    subsim = Subsim()
    rospy.spin()
