#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('UTDchess_RospyXbee')
import tf
import math
from geometry_msgs.msg import PointStamped
from UTDchess_RospyXbee.msg import Command
import time 

bot_origin = [0,0]


def get_target():
	#transforms the target to the robot's coordinate frame for the PI controller
	rospy.init_node('tf_listener', anonymous=True)
	listener = tf.TransformListener()
	point = PointStamped()
	point.point.x = 500
	point.point.z = 0
	point.header.frame_id = '/world'

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/world', '/chessbot', rospy.Time(0))
			target = listener.transformPoint('/chessbot', point)
			return target
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	

def build_vector(target, current):
	#builds a vector between two points
	vect = []
	vect[0] = calc_angle(target, current)
	vect[1] = calc_mag(target, current)
	return vect

def _clockwise(vect):
	if math.sin(vect[0]) > 0:
		return 0
	else:
		return 1


def calc_angle(target, current):
	#calculates the direction of a vector relative to a coordinate frame (origin given by current)
	deltaX = target[0] - current[0]
	deltaY = target[1] - current[1]
	angle = atan2(deltaY, deltaX) * 180 / PI
	return angle

def calc_mag(target, current):
	#calculates a vector's magnitude relative to a coordinate frame (origin given by current)
	mag = (target[1] - current[1])/(target[0] - current [0])
	return mag

def proportion_controller(target, current):
	#path[0] is direction, path[1] is magnitude
	cmd_pub = rospy.Publisher("/cmd_hex", Command)
	rospy.init_node('PID_contr', anonymous = True)
	rate = rospy.Rate(50)

	path = build_vector(target, current)

	#proportional constant
	k = 255/180

	while path[0] != 0:
		turn_speed = k * path[0]
		turn_direction = _clockwise(path)
		command = pack_msg(turn_direction, turn_speed)
		cmd_pub.publish(command)
	command = pack_msg(0, 0)
	cmd_pub.publish(command)


def pack_msg(td, ts):
	str(unichr())
	struct.pack('ccccc', str(unichr(4)), str(unichr(0)), str(unichr(0)), str(unichr(td)), str(unichr(ts)))


if __name__ == '__main__':
	try:
		proportion_controller(get_target, bot_origin)
	except rospy.ROSInterruptException:
		pass