#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
#Initialized ros parameters and global values
rospy.set_param('LINEAR_FORCE', 10)
rospy.set_param("TORQUE", 5)
rospy.set_param('ForceDown', -20)
rospy.set_param('TimeOfForceDown', 10)
Linear_Drag_X = 0
Linear_Drag_Y = 0
Linear_Drag_Z = 0
Roll_Drag = 0
Pitch_Drag = 0
Yaw_Drag = 0
Velocity = 0
''' 
Velocities between two time intervals will never be truly 'equal' in real world
Must compare the abs of the difference of the two with a small delta value
if the difference is less than this value we can conclude that the numbers are satisfyingly close 
also used as a value that is satisfyingly close to zero. Hard-coded 
'''
delta = .00001
#Max velocity cannot be initialized to 0 or there will be a divid-by-zero error
Max_Velocity = .1
Bouyancy = 0
appliedLinearForce = rospy.get_param("LINEAR_FORCE", default =10)
appliedTorque = rospy.get_param("TORQUE", default=5)
appliedForceDown = rospy.get_param('ForceDown', default = -20)
TimeOfAplliedForceDown = rospy.get_param('TimeOfForceDown')


def Find_Bouyancy(choice):
	global Bouyancy
	Down_Force = -.5
	pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=30)
	rospy.init_node('move_sub', anonymous=False)
	force_msg = WrenchStamped()
	force_msg.wrench.force.z = appliedForceDown
	pub.publish(force_msg)
	rospy.sleep(TimeOfAplliedForceDown)
	force_msg.wrench.force.z = 0
	pub.publish(force_msg)
	while not (rospy.is_shutdown()):
		rospy.Subscriber("/odom", Odometry, get_velocity, choice)	
		rospy.sleep(1)
		if Velocity > 0.0:
			while not (rospy.is_shutdown()):
				force_msg.wrench.force.z = Down_Force
				pub.publish(force_msg)
				Down_Force = Down_Force - .001
				rospy.sleep(.01)
				if Velocity < 0.0:
					break
			break
	Bouyancy = abs(Down_Force)



def get_velocity(data,choice):
	global Velocity
	if choice == 'x':
		Velocity = data.twist.twist.linear.x
	elif choice == 'y':
		Velocity = data.twist.twist.linear.y
	elif choice == 'z':
		Velocity = data.twist.twist.linear.z
	elif choice == 'rl':
		Velocity = data.twist.twist.angular.x
	elif choice == 'p':
		Velocity = data.twist.twist.angular.y
	elif choice == 'yw':
		Velocity = data.twist.twist.angular.z



def Calculate_Drag(choice):
	global Linear_Drag_X, Linear_Drag_Y, Linear_Drag_Z, Pitch_Drag, Roll_Drag, Yaw_Drag, Max_Velocity, Bouyancy
	if (choice == 'x'):
		Linear_Drag_X = (appliedLinearForce / abs(Max_Velocity))
	elif (choice == 'y'):
		Linear_Drag_Y = (appliedLinearForce / abs(Max_Velocity))
	elif (choice == 'z'):
		Linear_Drag_Z = ((appliedLinearForce - Bouyancy) / (abs(Max_Velocity)))
	elif (choice == 'rl'):
		Roll_Drag = (appliedTorque / abs(Max_Velocity))
	elif (choice == 'p'):
		Pitch_Drag = (appliedTorque / abs(Max_Velocity))
	elif (choice == 'yw'):
		Yaw_Drag = (appliedTorque / abs(Max_Velocity))


def Apply_Force(choice):
	global Max_Velocity, Bouyancy, Linear_Drag_Z
	pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=20)
	rospy.init_node('move_sub', anonymous=False)
	force_msg = WrenchStamped()
	if(choice == 'x'):
		force_msg.wrench.force.x = appliedLinearForce
	elif(choice == 'y'):
		force_msg.wrench.force.y = appliedLinearForce
	elif(choice == 'z'):
		force_msg.wrench.force.z = -appliedLinearForce
	elif(choice == 'yw'):
		force_msg.wrench.torque.z = appliedTorque
	elif(choice == 'rl'):
		force_msg.wrench.torque.x = appliedTorque
	elif(choice == 'p'):
		force_msg.wrench.torque.y = appliedTorque
	pub.publish(force_msg)
	
	while not (rospy.is_shutdown()):
		rospy.Subscriber("/odom", Odometry, get_velocity, choice)
		velocity1 = Velocity
		rospy.sleep(2)
		velocity2 = Velocity
		Compare_Velocities = abs(velocity1 - velocity2)
		if Compare_Velocities < delta: 
			Max_Velocity = velocity2
			Calculate_Drag(choice)
			break
		
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
	while(Velocity > delta and not rospy.is_shutdown()):
		continue


if __name__== '__main__':
	try:
		Find_Bouyancy('z')
		Apply_Force('z')
		Apply_Force('y')
		Apply_Force('x')
		Apply_Force('yw')
		Apply_Force('rl')
		Apply_Force('p')
		file_object = open("DragCoefficients", 'w')
		file_object.write("Drag Coefficients:")
		file_object.write("\nLinear X: " + str(Linear_Drag_X))
		file_object.write("\nLinear Y: " + str(Linear_Drag_Y))
		file_object.write("\nLinear Z: " + str(Linear_Drag_Z))
		file_object.write("\nRoll: " + str(Roll_Drag))
		file_object.write("\nPitch: " + str(Pitch_Drag))
		file_object.write("\nYaw: " + str(Yaw_Drag))
		file_object.close()
	except rospy.ROSInterruptException:
		pass