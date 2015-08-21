#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PointStamped
from chessbot.msg import RobCMD
from chessbot.msg import Vector
import time 
import threading


#this gives the transform listener the correct frame subscription for every node
NAME = rospy.get_namespace()
NAME = NAME[1:(len(NAME)-1)]
print "Bot: %s" % NAME

#initializing the global variables, the target point and repulsion vector
point = PointStamped()
repulsion = [0, 0]
obst = PointStamped()
mode = 0


def get_point():
	#This retrieves the bot's target point from the parameter server
	global NAME
	global point

	received_pt = rospy.get_param("/%s_point" % NAME)
	point.point.x = received_pt[0]
	point.point.y = received_pt[1]
	point.point.z = 0
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

def repulsion_subscriber():
	#This is started in its own thread to update the repulsion constant
	global NAME
	rospy.Subscriber("/%s/repulsions" % NAME, Vector, repulsion_calc)	
	rospy.spin()

def repulsion_calc(data):
	#This function creates an APF to 'repel' robots away from one another.
	global repulsion
	global obst
	global NAME
	global mode
	#the obstacle's location in the /world frame
	#print "x: %r" %data.end_x
	x2 = data.end_x
	y2 = data.end_y
	obst = PointStamped()
	obst.point.x = x2
	obst.point.y = y2
	#Transforms to the robot's frame
	obst.header.frame_id = '/world'
	rate = rospy.Rate(1000)
	while not rospy.is_shutdown():
		try:
			obst = listener.transformPoint('/%s' % NAME, obst)
			break
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rate.sleep()
			continue
	R = calc_mag([0, 0], [obst.point.x, obst.point.y])
	#print "R: %r" %R
	rV = [(-obst.point.x)/(.75*abs(R)), (-obst.point.y)/(.75*abs(R))]
	#print "rV: %r" % rV
	repulsion = rV
	#This constant adjusts how quickly the robot adjusts position to new points
	time.sleep(0.05)

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
	#This creates a new thread to update the repulsion variable
	sub = threading.Thread(target = repulsion_subscriber)
	sub.start()
	#Setting up the command publisher
	cmd_pub = rospy.Publisher("cmd_hex", RobCMD, queue_size=100)
	rate = rospy.Rate(10.0)
	#These are the constants used in the PI controller
	
	ki = 0.3
	integral_ang = 0

	try:
		while not rospy.is_shutdown():
			kp = 1.5
			#Accounting for all global variables
			global repulsion
			global obst
			#print "Current repulse: %r" % repulsion
			command = RobCMD()
			point = get_target()
			#path[0] is direction, path[1] is magnitude\
			print point.point.x
			print point.point.y
			#Adjusts the position of the robot's destination when it needs to
			#avoid another robot
			if abs(calc_mag([point.point.x, point.point.y], [0,0])) > abs(calc_mag([obst.point.x, obst.point.y], [0,0])):

				path = build_vector([(point.point.x + repulsion[0]) * 1000, (point.point.y + repulsion[1]) * 1000], [0,0])
			else:
				path = build_vector([point.point.x * 1000, point.point.y * 1000], [0,0])	


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
					print "HERE"
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