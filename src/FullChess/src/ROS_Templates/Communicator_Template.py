#!/usr/bin/env python
import sys
import rospy
import serial
import struct
import binascii
import time
from chessbot.msg import RobCMD
from chessbot.msg import BeeCommand

#from xbee import ZigBee

#xbee = None
#this ensures that each node can only communicate with one robot
NAME = rospy.get_namespace()
NAME = NAME[1:(len(NAME)-1)]
XBEE_ADDR_LONG = rospy.get_param("/%s_long" % NAME)
XBEE_ADDR_SHORT = rospy.get_param("/%s_short" % NAME)

#DEVICE = '/dev/ttyUSB0'
publ = rospy.Publisher("/cmd_hex", BeeCommand, queue_size=10)


def cmd_vel_command(msg):
    #sends the message through the coordinator Xbee, to the robot
    global publ
    global NAME
    finalCmd = BeeCommand()
    finalCmd.addr_long = XBEE_ADDR_LONG
    finalCmd.addr_short = XBEE_ADDR_SHORT
    finalCmd.command = msg
    rospy.loginfo(NAME)
    rospy.loginfo("Sending: %s" % finalCmd)
    publ.publish(finalCmd)
    #xbee.tx(
    #    dest_addr_long = XBEE_ADDR_LONG,
    #    dest_addr = XBEE_ADDR_SHORT,
    #    data=data,
    #)
    #print parse(xbee.wait_read_frame())

def callback(msg):
    cmd_vel_command(msg)

def listener():
    #initializes the subscriber that receives the movement commands
    print "Starting Listener"
    #global xbee

    #ser = serial.Serial(DEVICE, 57600)
    #xbee = ZigBee(ser)
    print "Ready to receive commands."
    rospy.init_node('Communicator', anonymous=False)
    
    rospy.Subscriber("cmd_hex", RobCMD, callback)
    rospy.spin()

    #xbee.halt()
    #ser.close()

def parse(frame):
    #Parses the transmit status on each message for relevant information
    info = {
        'length': str(len(frame)/2),
        'frame_id': frame[0:2],
        'addr': frame[2:8],
        'retry': frame[8:10],
        'status': frame[10:12]
        }
    return info



if __name__ == '__main__':
    listener()

