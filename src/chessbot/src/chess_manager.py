#!/usr/bin/env python
import rospy
from vicon_bridge.msg import Markers, Marker
from chessbot.msg import RobCMD
from chessbot.msg import Axis
from chessbot.msg import Vector
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import roslaunch
import time
import math

#Contains each bot's location, indexed by subject name
bot_locations = {}

#Node's publishers, indexed by namespace
bot_publishers = {}
bot_vectors_pubs = {}

def loc_callback(data):
	global bot_locations
	global bot_vectors_pubs
	prevIdent = data.markers[0].subject_name
	for marker_pos in data.markers:
		ident = marker_pos.subject_name
		#Tells the prorgam that a new message is beginning
		if ident != prevIdent:
			subdict = {prevIdent : [middle, front]}
			bot_locations.update(subdict)
			if prevIdent in bot_publishers.keys():
				send_location(prevIdent, bot_publishers[prevIdent])
		if marker_pos.marker_name == 'Mid':
			middle = [marker_pos.translation.x/1000, marker_pos.translation.y/1000]
		if marker_pos.marker_name == 'Front':
			front = [marker_pos.translation.x/1000, marker_pos.translation.y/1000]
		prevIdent = ident
	#So no off-by-one error
	subdict = {prevIdent : [middle, front]}
	bot_locations.update(subdict)
	if prevIdent in bot_publishers.keys():
		send_location(prevIdent, bot_publishers[prevIdent])

	#This sends messages to bots depending on where the other bots are in relation to them 
	#So said bots are able to avoid collisions using a PID-Repulsion controller
	for bot in bot_locations.keys():
		mag = 100000
		if bot == '':
			continue
		pub = bot_vectors_pubs[bot]
		vector = Vector()
		for other_bot in bot_locations:
			if other_bot != bot:
				if other_bot == '':
					continue
				if mag > calc_mag([bot_locations[bot][0][0], bot_locations[bot][0][1]], [bot_locations[other_bot][0][0], bot_locations[other_bot][0][1]]):
					mag = abs(calc_mag([bot_locations[bot][0][0], bot_locations[bot][0][1]], [bot_locations[other_bot][0][0], bot_locations[other_bot][0][1]]))
					vector.origin_x = bot_locations[bot][0][0]
					vector.origin_y = bot_locations[bot][0][1]
					vector.end_x = bot_locations[other_bot][0][0]
					vector.end_y = bot_locations[other_bot][0][1]
		#print bot
		#print vector 
		pub.publish(vector)

def calc_mag(target, current):
	#calculates a vector's magnitude relative to a coordinate frame (origin given by current)
	mag = (target[1] - current[1])**2 + (target[0] - current [0])**2
	mag = math.sqrt(mag)
	return mag

def send_location(bot, publisher):
	#sends the robot's TF broadcaster its current location
	vector = bot_locations[bot]

	axis = Axis()
	axis.center.x = vector[0][0]
	axis.center.y = vector[0][1]

	axis.front.x = vector[1][0]
	axis.front.y = vector[1][1]
	
	publisher.publish(axis)

def topic_creator(bot):
	#creates a topic for each robot's TF broadcaster in that robot's namespace
	global bot_publishers
	global bot_vectors_pubs
	bot_publishers[bot] = rospy.Publisher("/%s/destination" % bot, Axis, queue_size=10)
	bot_vectors_pubs[bot] = rospy.Publisher("/%s/repulsions" % bot, Vector, queue_size=10)

def node_creator(ns):
	topic_creator(ns)
	#initializes all necessary nodes in a robot's namespace
	package = 'FullChess'
	control_pkg = rospy.get_param("%s_controller" % ns)
	controller = roslaunch.core.Node(package, control_pkg, output = "screen", namespace=ns, launch_prefix="xterm -e")
	communicator = roslaunch.core.Node(package, 'Communicator_Template.py', namespace=ns, launch_prefix="xterm -e")
	broadcaster = roslaunch.core.Node(package, 'Broadcaster_template.py', namespace=ns, launch_prefix="xterm -e")
	launch = roslaunch.scriptapi.ROSLaunch()
	launch.start()

	#launches each node
	process = launch.launch(controller)
	process = launch.launch(communicator)
	process = launch.launch(broadcaster)

def get_addrs():
	global bot_publishers
	#finds each robot's name and address, and stores them as a global parameter
	xbee_coord = roslaunch.core.Node('UTDchess_RospyXbee', 'cmd_vel_listener.py', output = "screen")
	launch = roslaunch.scriptapi.ROSLaunch()
	launch.start()
	process = launch.launch(xbee_coord)

	data = rospy.wait_for_message("/bot_addrs", String)
	while data.data != 'end':
		addr_long = data.data[0:16]
		addr_short = data.data[16:20]
		name = "chessbot%s" % data.data[20:]
		rospy.set_param('%s_long' % name, addr_long)
		rospy.loginfo(rospy.get_param('%s_long' % name))
		rospy.set_param('%s_short' % name, addr_short)
		topic_creator(name)
		data = rospy.wait_for_message("/bot_addrs", String)
	for ns in bot_publishers.keys():
		node_creator(ns)

from chessbot.msg import RobCMD
if __name__ == '__main__':
	rospy.init_node('bot_locs_listener', anonymous=True)
	get_addrs()
	node_creator('chessbot32')
	node_creator('chessbot20')
	rospy.Subscriber("vicon/markers", Markers, loc_callback)
	rospy.spin()