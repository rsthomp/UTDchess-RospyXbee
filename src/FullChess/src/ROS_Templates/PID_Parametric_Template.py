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


#this gives the transform listener thesub correct frame subscription for every node
NAME = rospy.get_namespace()
NAME = NAME[1:(len(NAME)-1)]
print "Bot: %s" % NAME
point = PointStamped()
t = rospy.get_param("/%s_parametricStart" % NAME)

def get_point():
	global t
	global NAME
	#Enter your equations here. Note that this is the world frame
	point.point.x = eval(rospy.get_param("/%s_xEq" % NAME))
	point.point.y = eval(rospy.get_param("/%s_yEq" % NAME))
	t = t + .03
	print point
	return point

def get_target():
	#transforms the target to the robot's coordinate frame for the PI controller
	global NAME
	point = get_point()
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


def proportion_controller():
	#Initializing the command publisher
	global point
	cmd_pub = rospy.Publisher("cmd_hex", RobCMD, queue_size=100)
	rate = rospy.Rate(10.0)
	#Setting the PI constants
	kp = 1.2
	ki = 0.1
	integral_ang = 0
	try:
		while not rospy.is_shutdown():
			point = get_target()
			command = RobCMD()
			path = build_vector([point.point.x * 1000, point.point.y * 1000], [0,0])
			print "Path vector: %r " % path
			angle = path[0]
			mag = path[1]
			proportional_ang = (kp * angle)
			integral_ang += (ki * angle)
			#Setting the turn speed
			if integral_ang >= 255:
				integral_ang = 255
			if integral_ang <= -255:
				integral_ang = -255

			turn_speed = proportional_ang + integral_ang 
			
			if turn_speed >= 255:
				turn_speed = 255
			if turn_speed <= -255:
				turn_speed = -255
			#Setting the turn to clockwise or counterclockwise
			turn_direction = _clockwise(path)
			rospy.loginfo(NAME)
			rospy.loginfo("Turning Speed: %r" %turn_speed)
			rospy.loginfo("Turning Direction: %r" %turn_direction)

			command.code = 4
			command.turn = turn_direction
			command.accel = int(abs(turn_speed))
			command.magnitude = 0
			command.direction = 0

			#The robot moves forward when it is facing the target
			if abs(angle) < 20:
				print "Rolling"
				command.magnitude = 100
			#The robot stops once it reaches the target
			if abs(mag) < 50:
				command.magnitude = 0
				command.direction = 0
				command.turn = 0
				command.accel = 0
				cmd_pub.publish(command)
				#while get_point() == point:
					#time.sleep(.5)
 
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