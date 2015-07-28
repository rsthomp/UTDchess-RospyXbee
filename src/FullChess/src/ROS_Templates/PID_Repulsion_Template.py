#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PointStamped
from chessbot.msg import RobCMD
from chessbot.msg import Vector
import time 
import struct
import binascii
import threading

#CHANGES I MADE THAT I NEED TO TEST AT SOME POINT:
#changed proportion_controller to stop clogging with stop messages
#added coulomb's law repulsion stuff (need to check charge values)
#(not to mention just general functionality)
#add Vector to the relevant Cmakes and Package files
#used threading XO
#


#this gives the transform listener the correct frame subscription for every node
NAME = rospy.get_namespace()
NAME = NAME[1:(len(NAME)-1)]

#initializing the global variables, the target point and repulsion vector
point = PointStamped()
repulsion = [0, 0]


def get_point():
	#This retrieves the bot's target point from the parameter server
	global NAME
	global point


	pt = rospy.get_param("/%s_point" % NAME)

	point.point.x = pt[0]
	point.point.y = pt[1]
	point.point.z = 0
	return point

def get_target():
	#transforms the target to the robot's coordinate frame for the PI controller
	global NAME
	global point
	point.header.frame_id = '/world'
	rate = rospy.Rate(1000)
	while not rospy.is_shutdown():
		try:
			target = listener.transformPoint('/%s' % NAME, point)
			return target
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rate.sleep()
			continue

def repulsion_subscriber():
	#This is started in its own thread to update the repulsion constant
	rospy.Subscriber("/%s/repulsions", Vector, repulsion_calc)	
	rospy.spin()

def repulsion_calc(data):
	#This function uses Coulomb's law to 'repel' robots away from one another.
	global repulsion_const
	x1 = data.origin_x
	y1 = data.origin_y
	x2 = data.end_x
	y2 = data.end_y
	origin = [x1, y1]
	end = [x2, y2]
	R = abs(calc_mag(end, origin))
	
	#Our charge and electrical constants
	q1 = 1
	q2 = 1
	e0 = 9

	#rV is the vector of the 'charge' that will be used to adjust the course of the robot
	rV = [((x2 - x1)/R**3)((q1*q2)/(4*math.pi*e0)), ((y2 - y1)/R**3)((q1*q2)/(4*math.pi*e0))]
	repulsion_const += rV

def build_vector(target, current):
	#builds a vector between two points
	vect = [0, 0]
	vect[0] = calc_angle(target, current)
	vect[1] = calc_mag(target, current)
	return vect

def _clockwise(vect):
	#Tells the robot whether it needs to turn clockwise or counterclockwise
	#For the fastest path to the correct heading
	if math.sin( (vect[0] * math.pi)/180 ) > 0:
		return 0
	else:
		return 1

def calc_angle(target, current):
	#calculates the direction of a vector relative to a coordinate frame (origin given by current)
	deltaX = target[0] - current[0]
	deltaY = target[1] - current[1]
	angle = math.atan2(deltaY, deltaX) * 180 / math.pi
	return angle

def calc_mag(target, current):
	#calculates a vector's magnitude relative to a coordinate frame (origin given by current)
	mag = (target[1] - current[1])**2 + (target[0] - current [0])**2
	mag = math.sqrt(mag)
	return mag


def proportion_controller(data):
	#This creates a new thread to update the repulsion variable
	sub = threading.Thread(target = repulsion_subscriber)
	sub.daemon = True
	sub.start
	#Setting up the command publisher
	cmd_pub = rospy.Publisher("cmd_hex", RobCMD, queue_size=100)
	rate = rospy.Rate(10.0)
	#These are the constants used in the PI controller
	kp = 1.5
	ki = 0.3
	integral_ang = 0

	try:
		while not rospy.is_shutdown():
			#Accounting for all global variables
			global repulsion
			command = RobCMD()
			point = get_target()
			#path[0] is direction, path[1] is magnitude
			path = build_vector([point.point.x * 1000 + repulsion[0], point.point.y * 1000 + repulsion [1]], [0,0])
			print "Path vector: %r " % path
			angle = path[0]
			mag = path[1]
			#Calculating the speed of the robot's turn based on the angle
			#to the target
			proportional_ang = (kp * angle)
			integral_ang += (ki * angle)

			if integral_ang >= 255:
				integral_ang = 255
			if integral_ang <= -255:
				integral_ang = -255

			turn_speed = proportional_ang + integral_ang 
			
			if turn_speed >= 255:
				turn_speed = 255
			if turn_speed <= -255:
				turn_speed = -255

			#Tells the robot to turn left or right
			turn_direction = _clockwise(path)

			rospy.loginfo(NAME)
			rospy.loginfo("Turning Speed: %r" %turn_speed)
			rospy.loginfo("Turning Direction: %r" %turn_direction)

			#Preparing the command to send the robot
			command.code = 4
			command.turn = turn_direction
			command.accel = int(abs(turn_speed))
			command.magnitude = 0
			command.direction = 0

			#Moves the robot forward if it is facing the target
			if abs(angle) < 20:
				command.magnitude = 100

			#Stops the robot when it reaches the target, and requests
			#the next target. 
			if abs(mag) < 50:
				command.magnitude = 0
				command.direction = 0
				command.turn = 0
				command.accel = 0
				cmd_pub.publish(command)
				while get_point() == point:
					time.sleep(.5)
 
			cmd_pub.publish(command)
			rate.sleep()

		rate.sleep()
	except KeyboardInterrupt:
		
		raise



if __name__ == '__main__':
	try:
		rospy.init_node('tf_listener', anonymous=False)
		listener = tf.TransformListener()
		get_point()
		proportion_controller()
	except rospy.ROSInterruptException, KeyboardInterrupt:
		raise