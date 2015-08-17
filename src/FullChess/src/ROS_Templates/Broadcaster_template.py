#!/usr/bin/env python
import rospy
from vicon_bridge.msg import Markers, Marker
import tf
import math
from std_msgs.msg import String
from chessbot.msg import Axis

#this makes sure all the transforms are labeled correctly for every node
NAME = rospy.get_namespace()
NAME = NAME[1:(len(NAME)-1)]
print "Bot: %s" % NAME

#pub = rospy.Publisher('/study')

def avg(data):
    global NAME
    center = [data.center.x, data.center.y]
    rospy.loginfo("Bot: %s" % NAME)
    rospy.loginfo("Bot position: %s" % center)

    #this is the Y axis
    BOT_X_AXIS = [[center[0], center[1]], [data.front.x, data.front.y]]


    BOT_ROTATION = calc_angle(BOT_X_AXIS[0], BOT_X_AXIS[1])

    #This should all be pretty much good
    br.sendTransform((center[0], center[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, BOT_ROTATION),
                     rospy.Time.now(),
                     NAME,
                     "world")

def tf_broadcaster():
        rospy.init_node('tf_broadcaster', anonymous = False)
        rospy.is_shutdown()
        rospy.Subscriber('destination', Axis, avg)
        rospy.spin()

def calc_angle(center, end):
    #calculates the angle between the X-axis of the world and chessbot frames
    deltaX = end[0] - center[0]
    deltaY = end[1] - center[1]
    angle = math.atan2(deltaY, deltaX)  
    return angle

if __name__ == '__main__':
    try:
        br = tf.TransformBroadcaster()
        tf_broadcaster()
    except rospy.ROSInterruptException:
        raise