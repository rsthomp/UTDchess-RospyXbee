#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PointStamped
from chessbot.msg import RobCMD
import time 
import struct
import binascii
import thread

bot_origin = [0,0]

#this gives the transform listener the correct frame subscription for every node
NAME = rospy.get_namespace()
NAME = NAME[1:(len(NAME)-1)]

def get_target():
	#transforms the target to the robot's coordinate frame for the PI controller
	global NAME
	
	point = PointStamped()

	#we need a way to get and update sample points. 
	point.point.x = 0
	point.point.y = 0
	point.point.z = 0


	point.header.frame_id = '/world'
	rate = rospy.Rate(1000)
	while not rospy.is_shutdown():
		try:
			target = listener.transformPoint('/%s' % NAME, point)
			return target
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rate.sleep()
			continue

	

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


def proportion_controller(target, current):
	#path[0] is direction, path[1] is magnitude
	cmd_pub = rospy.Publisher("cmd_hex", RobCMD, queue_size=100)
	#rospy.init_node('PID_contr', anonymous = True)
	rate = rospy.Rate(10.0)
	#proportional constant
	kp = 2.5
	ki = 0.1
	integral_ang = 0
	integral_mag = 0
	prevAngle = 0
	path = build_vector(target, current)
	try:
		while path[0] != 0 and not rospy.is_shutdown():
			command = RobCMD()
			point = get_target()
			target[0] = point.point.x * 1000
			target[1] = point.point.y *1000
			path = build_vector(target, current)
			print "Path vector: %r " % path
			angle = path[0]
			proportional_ang = (kp * angle)
			integral_ang += (ki * angle)
			turn_speed = proportional_ang + integral_ang 
			

			if integral_ang >= 255:
				integral_ang = 255
			if integral_ang <= -255:
				integral_ang = -255
			if integral_mag >= 255:
				integral_mag = 255
			if integral_mag <= -255:
				integral_mag = -255
			if turn_speed >= 255:
				turn_speed = 255
			if turn_speed <= -255:
				turn_speed = -255

			

			prevAngle = angle
			turn_direction = _clockwise(path)

			rospy.loginfo("Turning Speed: %r" %turn_speed)
			rospy.loginfo("Turning Direction: %r" %turn_direction)

			command.code = 4
			command.turn = turn_direction
			command.accel = int(abs(turn_speed))
			command.magnitude = 0
			command.direction = 0

			if path[0] < 10 and path[0] > -10:
				command.turn = 0
				command.accel = 0
				command.magnitude = 100
				command.direction = 0
			if path[1]*1000 < 50 and path[1]*1000 > -50:
				command.magnitude = 0
				command.direction = 0
 
			cmd_pub.publish(command)
			rate.sleep()

		rate.sleep()
	except KeyboardInterrupt:
		
		raise



if __name__ == '__main__':
	try:
		rospy.init_node('tf_listener', anonymous=False)
		listener = tf.TransformListener()
		target_pt = [0,0]
		target_pt[0] = get_target().point.x
		target_pt[1] = get_target().point.y
		proportion_controller(target_pt, bot_origin)
	except rospy.ROSInterruptException, KeyboardInterrupt:
		raise