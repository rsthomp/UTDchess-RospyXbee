#!/usr/bin/env python
import rospy
from vicon_bridge.msg import Markers, Marker
from chessbot.msg import RobCMD
from chessbot.msg import Axis
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import binascii
from math import sin, cos, atan2, pi
import roslaunch
import time

#Contains each bot's location, indexed by subject name
bot_locations = {}

#Node's topics, indexed by namespace
bot_publishers = {}
processes = []


def loc_callback(data):
	global bot_locations
	prevIdent = data.markers[0].subject_name
	for marker_pos in data.markers:
		ident = marker_pos.subject_name
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
	subdict = {prevIdent : [middle, front]}
	bot_locations.update(subdict)
	if prevIdent in bot_publishers.keys():
		send_location(prevIdent, bot_publishers[prevIdent])

def find_bot(bot):
	pass

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
	bot_publishers[bot] = rospy.Publisher("/%s/destination" % bot , Axis, queue_size=100)

def node_creator(ns):
	topic_creator(ns)
	#initializes all necessary nodes in a robot's namespace
	package = 'FullChess'
	controller = roslaunch.core.Node(package, 'PID_Template.py', namespace=ns, launch_prefix="xterm -e")
	communicator = roslaunch.core.Node(package, 'Communicator_Template.py', namespace=ns, launch_prefix="xterm -e", output="screen")
	broadcaster = roslaunch.core.Node(package, 'Broadcaster_template.py', namespace=ns, launch_prefix="xterm -e")
	launch = roslaunch.scriptapi.ROSLaunch()
	launch.start()

	
	process = launch.launch(controller)
	processes.append(process)
	process = launch.launch(communicator)
	processes.append(process)
	process = launch.launch(broadcaster)
	processes.append(process)
	

	
	

def get_addrs():
	#finds each robot's name and address, and stores them as a global parameter
	package = 'UTDchess_RospyXbee'
	node_discover = roslaunch.core.Node(package, 'cmd_vel_listener.py', output = "screen", launch_prefix="xterm -e")
	launch = roslaunch.scriptapi.ROSLaunch()
	launch.start()
	process = launch.launch(node_discover)
	data = rospy.wait_for_message("/bot_addrs", String)
	while data.data != 'end':
		addr_long = data.data[0:16]
		addr_short = data.data[16:20]
		name = "chessbot%s" % data.data[20:]
		rospy.set_param('%s_long' % name, addr_long)
		rospy.loginfo(rospy.get_param('%s_long' % name))
		rospy.set_param('%s_short' % name, addr_short)
		node_creator(name)
		data = rospy.wait_for_message("/bot_addrs", String)
	

from chessbot.msg import RobCMD
if __name__ == '__main__':
	rospy.init_node('bot_locs_listener', anonymous=True)

	get_addrs()
	rospy.Subscriber("vicon/markers", Markers, loc_callback)
	rospy.spin()
	global processes
	for process in processes:
		process.end()
#publish struct.pack( 'q', XBEE_ADDR_LONG)locations and addresses to a parameter dictionary
#may need to give nodes generic names then re-resolve them. 