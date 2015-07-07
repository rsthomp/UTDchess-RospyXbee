#!/usr/bin/env python
import roslib
#roslib.load_manifest('learning_tf')
import rospy
from geometrymsgs import TransformedStamp
import tf

def avg(data):
    crayola = data.markers[1]
    rospy.loginfo("Stuff: %r "data.translation)


def vicon_listener():
    rospy.init_node('vicon_listener', anonymous = True)
    rospy.subscriber('vicon/markers', TransformedStamp, avg)