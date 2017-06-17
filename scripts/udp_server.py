#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import sys
import socket
import time
import struct

from ccn_final.msg import RaspiCarCamera
from ccn_final.msg import RaspiCarDistance
from ccn_final.msg import RaspiCarWheel

Frame_Width = 320
Frame_Height = 240

g_key = None
g_frame = None
g_proximity = np.zeros([3])
g_wheel_count = np.zeros([2])


def callback_RaspiCarCamera(data):
    # rospy.loginfo('%s, %d' % (data.info, data.time_stamp))

    global g_frame
    g_frame = data.camera_jpg

    global g_key
    g_key = cv2.waitKey(1) & 0xFF


def callback_RaspiCarDistance(data):
    # rospy.loginfo('%s, %d, distance = [%3d,%3d,%3d]' % (data.info, data.time_stamp,
    #                                                     data.distance[0], data.distance[1], data.distance[2]))
    global g_proximity
    g_proximity = data.distance


def callback_RaspiCarWheel(data):
    # rospy.loginfo('%s, %d, wheel_count = [%6d,%6d]' % (data.info, data.time_stamp,
    #                                                    data.wheel_count[0], data.wheel_count[1]))
    global g_wheel_count
    g_wheel_count = data.wheel_count


def listener():
    rospy.init_node('RaspiCarViewTopics_node', anonymous=False)
    rospy.Subscriber('RaspiCarCamera', RaspiCarCamera, callback_RaspiCarCamera)
    rospy.Subscriber('RaspiCarDistance', RaspiCarDistance, callback_RaspiCarDistance)
    rospy.Subscriber('RaspiCarWheel', RaspiCarWheel, callback_RaspiCarWheel)


def udp_routine():
    target_ip = None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0)
    try:
        server_address = ('', 23232)
        rospy.loginfo('[udp_server] binding to %s port %s' % server_address)
        sock.bind(server_address)

        global g_frame
        while not rospy.is_shutdown():

            try:
                (packet, ip) = sock.recvfrom(128)
                if packet == b'HELLO':
                    target_ip = ip
            except socket.error:
                # rospy.loginfo('[udp_server] wait for HELLO message...')
                pass

            if target_ip is not None and g_frame is not None:
                frame = g_frame
                g_frame = None
                time_stamp = int(time.clock() * 1000)
                header = struct.pack('qHHHHII', time_stamp, len(frame), g_proximity[0], g_proximity[1],
                                     g_proximity[2], g_wheel_count[0], g_wheel_count[1])
                sock.sendto((header + frame), target_ip)
                # rospy.loginfo('[udp_server] sending data%06d(%d bytes) to %s. ' % (time_stamp, len(frame), target_ip[0]))
                print('[udp_server] sending data%06d(%d bytes) to %s. ' % (time_stamp, len(frame), target_ip[0]))

            else:
                time.sleep(0.01)

    except socket.error as e:
        print(e)
    finally:
        sock.close()
        print('[udp_server] stopped.')


if __name__ == '__main__':
    rospy.loginfo(sys.version)
    rospy.loginfo(cv2.__version__)

    try:
        listener()
        udp_routine()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)
        pass
