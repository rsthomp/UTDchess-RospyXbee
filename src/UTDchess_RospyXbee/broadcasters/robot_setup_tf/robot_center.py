#!/usr/bin/env python
import rospy
from vicon_bridge.msg import Markers, Marker
import tf
import math


def avg(data):
    Right = data.markers[0]
    Mid = data.markers[1]
    Left = data.markers[2]
    Front = data.markers[3]

    #this is center



    center = [Mid.translation.x, Mid.translation.y]

    #this is the Y axis
    BOT_X_AXIS = [[center[0], center[1]], [Front.translation.x, Front.translation.y]]


    BOT_ROTATION = calc_angle(BOT_X_AXIS[0], BOT_X_AXIS[1])

    #X axis is going to be through
    br.sendTransform((center[0]/1000, center[1]/1000, 0),
                     tf.transformations.quaternion_from_euler(0, 0, BOT_ROTATION),
                     rospy.Time.now(),
                     "chessbot",
                     "world")

def vicon_listener():
    rospy.init_node('vicon_listener', anonymous = True)
    rospy.Subscriber('vicon/markers', Markers, avg)
    rospy.spin()

def calc_angle(center, end):
    deltaX = end[0] - center[0]
    deltaY = end[1] - center[1]
    print "DX: %r" %deltaX
    print "DY: %r" %deltaY
    #print "angle: %r" % angle
    angle = math.atan2(deltaY, deltaX)
    print angle * 180 / math.pi    
    return angle

if __name__ == '__main__':
    br = tf.TransformBroadcaster()

    vicon_listener()