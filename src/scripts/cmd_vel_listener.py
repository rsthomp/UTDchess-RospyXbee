#!/usr/bin/env python
import sys
import rospy
import serial
import struct
import binascii
import time
from teleop_twist_keyboard.msg import Command

from xbee import ZigBee

xbee = None
XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x40\x86\x96\x4F'
XBEE_ADDR_SHORT = '\xFF\xFE'
DEVICE = '/dev/tty.usbserial-A603HA9K'
#Each bot will have an addr long, addr short, and id stored. Coordinator is always id 0.
bot_array = []
final_bot_array = []


def find_bots():
    #The coordinator broadcasts a "Node Discover" AT command and records the addresses recieved, I suspect
    #Add coordinator first, then add on each bot as it responds.
    #robot id command
    
    global xbee
    
    ser = serial.Serial(DEVICE, 57600)
    xbee = ZigBee(ser)

    try:
        print("Searching for bots...")
        xbee.at(
            dest_addr_long = XBEE_ADDR_LONG,
            dest_addr = XBEE_ADDR_SHORT,
            command = 'ND'
            )
        timeout = time.time() + 30
        num_of_robots = 0
        while timeout > time.time():
            dict = xbee.wait_read_frame()
            if dict == None:
                break
            bot_array.append(parse_ND(dict))
            print "Response: %r " % bot_array[num_of_robots]
            num_of_robots += 1
    except KeyboardInterrupt, SerialException:
        sys.exit(0)

def get_bot_id(addr_long, addr_short):
    #STILL NEEDS WORK
    data = struct.pack('c', str(unichr(2)))
    for item in bot_array:
        addr_long = hex_to_addr(item['addr_long'])
        addr_short = hex_to_addr(item['addr_short'])
        xbee.tx(
            dest_addr_long = addr_long,
            dest_addr = addr_short,
            data=data,
            )
        ans = xbee.wait_read_frame()
        if ans == None:
            print "Could not retrieve bot id"
        print ans[6:12]


def search_for_bot(id):
    #Takes the xbee ID, returns the xbee's dictionary
    for dict in bot_array:
        if dict['id'] == id:
            return dict



def prepare_move_cmd(msg):
    code = str(unichr(4))
    
    #Packages the command message as binary for the API Frame
    move = str(unichr(msg.direction))
    speed = str(unichr(msg.magnitude))
    turn = str(unichr(msg.turn))
    accel = str(unichr(msg.accel))
    data = struct.pack('ccccc', code, move, speed, turn, accel)
    
    return data

def cmd_vel_command(msg):
    data = prepare_move_cmd(msg)

    rospy.loginfo("Sending: %s" % binascii.hexlify(data))
    
    #Sends the message
    xbee.tx(
        dest_addr_long = XBEE_ADDR_LONG,
        dest_addr = XBEE_ADDR_SHORT,
        data=data,
    )
    print parse(xbee.wait_read_frame())

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    cmd_vel_command(msg)

def listener():
    global xbee

    ser = serial.Serial(DEVICE, 57600)
    xbee = ZigBee(ser)

    # in ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    print "Ready to receive commands."
    rospy.init_node('cmd_vel_listener', anonymous=True)
    
    rospy.Subscriber("/cmd_hex", Command, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    xbee.halt()
    ser.close()

def parse(frame):
    #Parses the transmit status for relevant info
    info = {
        'length': str(len(frame)/2),
        'frame_id': frame[0:2],
        'addr': frame[2:8],
        'retry': frame[8:10],
        'status': frame[10:12]
        }
    return info

def parse_ND(frame):
    #parses the node discovery response for relevant info
    info = {
        'length': str(len(frame)/2),
        'frame_id': frame[0:2],
        'command': frame[2:6],
        'status': frame[6:8],
        'addr_short': frame[8:12],
        'addr_long': frame[12:28],
        'id':  frame[39:40]
        }
    return info

def hex_to_addr(adhex):
    #Changes the hex address given by the dictionary
    #to a format usable by the xbee. Works on long and short.
    address = binascii.unhexlify(adhex)
    return address


def send_move_command(id, msg)
    dict = search_for_bot(id)
    data = prepare_move_cmd(msg)
    
    rospy.loginfo("Sending: %s" % binascii.hexlify(data))
    
    #Sends the message
    xbee.tx(
            dest_addr_long = unhexlify(dict['addr_long']),
            dest_addr = unhexlify(dict['addr_short']),
            data=data,
            )
    rospy.loginfo("   ")parse(xbee.wait_read_frame())



if __name__ == '__main__':
    find_bots()
    #print search_for_bot('1')
    #listener()
    get_bot_id(XBEE_ADDR_LONG, XBEE_ADDR_SHORT)


#tf package


