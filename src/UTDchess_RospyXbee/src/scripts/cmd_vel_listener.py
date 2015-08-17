#!/usr/bin/env python
import sys
import rospy
import serial
import struct
import binascii
import time
from std_msgs.msg import String
from chessbot.msg import BeeCommand
from xbee import ZigBee


xbee = None
XBEE_ADDR_LONG = '\x00\x00\x00\x00\x00\x00\xFF\xFF'
XBEE_ADDR_SHORT = '\xFF\xFE'
DEVICE = '/dev/ttyUSB0'
#Each bot will have an addr long, addr short, and id stored. Coordinator is always id 0.
bot_array = []
final_bot_array = []


def exit_handler():
    stop = BeeCommand()
    stop.command.direction = 0
    stop.command.magnitude = 0
    stop.command.turn = 0
    stop.command.accel = 0
    ser = serial.Serial(DEVICE, 57600)
    xbee = ZigBee(ser)
    xbee.tx(
        dest_addr_long = XBEE_ADDR_LONG,
        dest_addr = XBEE_ADDR_SHORT,
        data = prepare_move_cmd(stop.command),
    )
    xbee.tx(
        dest_addr_long = XBEE_ADDR_LONG,
        dest_addr = XBEE_ADDR_SHORT,
        data = prepare_move_cmd(stop.command),
    )
    xbee.tx(
        dest_addr_long = XBEE_ADDR_LONG,
        dest_addr = XBEE_ADDR_SHORT,
        data = prepare_move_cmd(stop.command),
    )
    

def find_bots():
    #The coordinator broadcasts a "Node Discover" AT command and records the addresses recieved, I suspect
    #Add coordinator first, then add on each bot as it responds.
    #robot id command
    
    global xbee
    
    ser = serial.Serial(DEVICE, 57600)
    xbee = ZigBee(ser)

    try:
        print("Searching for bots. This may take a moment.")
        xbee.at(
            dest_addr_long = XBEE_ADDR_LONG,
            dest_addr = XBEE_ADDR_SHORT,
            command = 'ND'
            )
        timeout = time.time() + 30
        num_of_robots = 0
        while timeout > time.time():
            dictl = xbee.wait_read_frame()
            if dictl == None:
                break
            bot_array.append(parse_ND(dictl))
            num_of_robots += 1
    except KeyboardInterrupt:
        sys.exit(0)

def assemble_msg(info):
    msg = ''
    msg += info['addr_long']
    msg += info['addr_short']
    msg += info['id']
    return msg


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
        'frame_id': frame[2:4],
        'command': frame[4:8],
        'status': frame[8:10],
        'addr_short': frame[10:14],
        'addr_long': frame[14:30],
        'id':  frame[30:32]
        }
    return info

def hex_to_addr(adhex):
    #Changes the hex address given by the dictionary
    #to a format usable by the xbee. Works on long and short.
    address = binascii.unhexlify(adhex)
    print "Address found: %s " % binascii.hexlify(address)
    return address

def prepare_move_cmd(msg):
    code = chr(4)
    
    #Packages the command message as binary for the API Frame
    move = chr(msg.direction)
    speed = chr(msg.magnitude)
    turn = chr(msg.turn)
    accel = chr(msg.accel)
    data = struct.pack('ccccc', code, move, speed, turn, accel)
    return data

def callback(data):
    xbee.tx(
        dest_addr_long = hex_to_addr(data.addr_long),
        dest_addr = hex_to_addr(data.addr_short),
        data = prepare_move_cmd(data.command),
    )
    print "#######################################################"
    rospy.log_info(data.command)


def listener():
    #initializes the subscriber that receives the movement commands
    global xbee

    ser = serial.Serial(DEVICE, 57600)
    xbee = ZigBee(ser)
    print "Coordinator ready to receive commands."
    
    rospy.Subscriber("/cmd_hex", BeeCommand, callback)
    rospy.spin()

    xbee.halt()
    ser.close()



if __name__ == '__main__':
    rospy.on_shutdown(exit_handler)
    find_bots()
    rospy.init_node('addr_publisher')
    pub = rospy.Publisher('/bot_addrs', String, queue_size=1)
    while not pub.get_num_connections() > 0:
        time.sleep(.5)
    for bot in bot_array:
        addr_msg = String()
        addr_msg = assemble_msg(bot)
        pub.publish(addr_msg)
        time.sleep(1)
    pub.publish('end')
    listener()



