#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
# Initialized ros parameters and global values (feel free to tweak, values were empirically gained through trial/error)


class Drag_C_Calc(object):

    def __init__(self):
        self.linear_drag_x = 0
        self.linear_drag_y = 0
        self.linear_drag_z = 0
        self.roll_drag = 0
        self.pitch_drag = 0
        self.yaw_drag = 0
        self.velocity = 0
        '''
        Velocities between two time intervals will never be truly 'equal' in real world
        Must compare the abs of the difference of the two with a small delta value
        if the difference is less than this value we can conclude that the numbers are satisfyingly close
        also used as a value that is satisfyingly close to zero. Hard-coded (can be changed).
        '''
        self.delta = .00001
        # Max velocity cannot be initialized to 0 or there will be an initial divide-by-zero error
        self.max_velocity = .1
        self.bouyancy = 0
        # Magnitude of applied force for determining drag coeffcients in linear axes
        self.applied_linear_force = rospy.get_param("linear_force", default=10)
        # Magnitude of applied torque for determining drag coeffcients in rotational axes
        self.applied_torque = rospy.get_param("torque", default=5)
        self.applied_force_down = rospy.get_param('force_down', default=-20)
        self.time_of_apllied_force_down = rospy.get_param('time_of_force_down', default=10)
        self.drag = {}

    def find_bouyancy(self, choice):  # Initial function used upward force of buoyancy(used determining drag in Z axis)
        down_force = -.5  # Applies initial downward force in z axis
        pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=30)
        rospy.init_node('move_sub', anonymous=False)
        force_msg = WrenchStamped()
        force_msg.wrench.force.z = self.applied_force_down
        pub.publish(force_msg)  # publish wrench with force
        rospy.loginfo('Moving sub down...')
        rospy.sleep(self.time_of_apllied_force_down)  # node sleeps for some amount of time before continuing
        force_msg.wrench.force.z = 0  # Applies 0 force which stops downward force
        pub.publish(force_msg)
        while not (rospy.is_shutdown()):
            rospy.Subscriber("/odom", Odometry, self.get_velocity, choice)
            rospy.sleep(1)
            if self.velocity > 0.0:
                rospy.loginfo('Appling a force down to calculate the bouyancy')
                while not (rospy.is_shutdown()):
                    force_msg.wrench.force.z = down_force
                    pub.publish(force_msg)
                    down_force = down_force - .001
                    rospy.sleep(.01)
                    if self.velocity < 0.0:
                        break
                rospy.loginfo('bouyancy found!')
                break
        self.bouyancy = abs(down_force)
        rospy.loginfo('Bouyancy: {}'.format(self.bouyancy))

    def get_velocity(self, data, choice):  # Function sets velocity in a certain axis depending on char input
        # linear
        if choice == 'x':
            self.velocity = data.twist.twist.linear.x
        elif choice == 'y':
            self.velocity = data.twist.twist.linear.y
        elif choice == 'z':
            self.velocity = data.twist.twist.linear.z
        # rotational
        elif choice == 'rl':
            self.velocity = data.twist.twist.angular.x
        elif choice == 'p':
            self.velocity = data.twist.twist.angular.y
        elif choice == 'yw':
            self.velocity = data.twist.twist.angular.z

    def calculate_drag(self, choice):
        '''
        Calculates drag based on the initial applied force and the approximate max velocity
        the sub achieves in that axis. See for formula: http://hyperphysics.phy-astr.gsu.edu/hbase/airfri.html
        Axis determined by char argument.
        '''
        if (choice == 'x'):
            self.linear_drag_x = (self.applied_linear_force / abs(self.max_velocity))
            self.drag['x:'] = self.linear_drag_x
        elif (choice == 'y'):
            self.linear_drag_y = (self.applied_linear_force / abs(self.max_velocity))
            self.drag['y:'] = self.linear_drag_y
        elif (choice == 'z'):
            # Buoyancy affects z axis and must be subtracted from applied for before division.
            self.linear_drag_z = ((self.applied_linear_force - self.bouyancy) / (abs(self.max_velocity)))
            self.drag['z:'] = self.linear_drag_z
        elif (choice == 'rl'):
            self.roll_drag = (self.applied_torque / abs(self.max_velocity))
            self.drag['roll:'] = self.roll_drag
        elif (choice == 'p'):
            self.pitch_drag = (self.applied_torque / abs(self.max_velocity))
            self.drag['pitch:'] = self.pitch_drag
        elif (choice == 'yw'):
            self.yaw_drag = (self.applied_torque / abs(self.max_velocity))
            self.drag['yaw:'] = self.yaw_drag

    def apply_force(self, choice):
        '''
        Function applies force in a given axis and allows sub to achieve
        terminal (linear/rotationl) velocity in that direction. Once that
        max velocity is found, it is used in the calculate_drag() function.
        '''
        global max_velocity, bouyancy, linear_drag_z
        pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=20)  # Publisher for applying wrench (force/torque)
        rospy.init_node('move_sub', anonymous=False)
        force_msg = WrenchStamped()
        # Char argument determines axis of applied force/torque
        if(choice == 'x'):
            force_msg.wrench.force.x = self.applied_linear_force
        elif(choice == 'y'):
            force_msg.wrench.force.y = self.applied_linear_force
        elif(choice == 'z'):
            force_msg.wrench.force.z = -self.applied_linear_force
        elif(choice == 'yw'):
            force_msg.wrench.torque.z = self.applied_torque
        elif(choice == 'rl'):
            force_msg.wrench.torque.x = self.applied_torque
        elif(choice == 'p'):
            force_msg.wrench.torque.y = self.applied_torque

        pub.publish(force_msg)
        rospy.loginfo('Appling a force to find the drag in the {} direction'.format(choice))
        while not (rospy.is_shutdown()):
            '''
            On each iteration of while loop: velocity is gained at two points in time (deltat = 2 seconds)
            the velocity is compared by taking the absolute value of the difference of the two velocities
            if the abs of the difference of the two velocities is smaller than small delta value,
            the two velocities are assumed to be approximately equal. If two velocities taken at different
            time intervals are equal, the velocity is assumed to be maximized, reaching terminal velocity.
            '''
            rospy.Subscriber("/odom", Odometry, self.get_velocity, choice)
            velocity1 = self.velocity
            rospy.sleep(2)
            velocity2 = self.velocity
            Compare_Velocities = abs(velocity1 - velocity2)
            if Compare_Velocities < self.delta:
                rospy.loginfo('Velocities are equal')
                self.max_velocity = velocity2
                self.calculate_drag(choice)  # Once velocity is max, calculate drag using max velocity
                rospy.loginfo('Linear Drag Values-- x: {}, y: {}'.format(self.linear_drag_x, self.linear_drag_y))
                rospy.loginfo('z: {}'.format(self.linear_drag_z))
                rospy.loginfo('Rotational Drag Values-- yaw: {}, pitch: {}'.format(self.yaw_drag, self.pitch_drag))
                rospy.loginfo('roll: {}'.format(self.roll_drag))
                break  # When the velocities are 'equal', the loop breaks.

        # Once drag is calculated, apply wrench with force/torque of zero to slow sub in that axis
        if(choice == 'x'):
            force_msg.wrench.force.x = 0
        elif(choice == 'y'):
            force_msg.wrench.force.y = 0
        elif(choice == 'z'):
            force_msg.wrench.force.z = 0
        elif(choice == 'yw'):
            force_msg.wrench.torque.z = 0
        elif(choice == 'rl'):
            force_msg.wrench.torque.x = 0
        elif(choice == 'p'):
            force_msg.wrench.torque.y = 0

        pub.publish(force_msg)
        rospy.loginfo('stopping sub')
        while(self.velocity > self.delta and not rospy.is_shutdown()):
            # While loop stops program from proceeding until the sub has basically stopped movement in that direction
            rospy.sleep(2)
            continue

    def fileOutput(self):
            f = open('drag_coefficients.yaml', 'w')
            yaml.dump(self.drag, f, default_flow_style=False)


if __name__ == '__main__':
    try:
        calculator = Drag_C_Calc()
        # Buoyancy is calculated
        calculator.find_bouyancy('z')
        # Drag Coeffcients are calculated in each axis
        calculator.apply_force('z')
        calculator.apply_force('y')
        calculator.apply_force('x')
        calculator.apply_force('yw')
        calculator.apply_force('rl')
        calculator.apply_force('p')
        # Drag coefficients are written to file, do with them what you wish.
        calculator.fileOutput()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
