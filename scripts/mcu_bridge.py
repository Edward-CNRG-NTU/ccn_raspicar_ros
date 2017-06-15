#!/usr/bin/env python

import rospy
import numpy as np
import time
import serial
import select

from ccn_final.msg import RaspiCarDistance
from ccn_final.msg import RaspiCarWheel
from ccn_final.msg import RaspiCarServo


def talker(serial_port):

    serial_port.write('R')

    rospy.init_node('RaspiCarMCUBridge_node', anonymous=False)

    rospy.Subscriber('RaspiCarServo', RaspiCarServo, lambda data: serial_port.write(chr(data.servo_control)))

    pub_distance = rospy.Publisher('RaspiCarDistance', RaspiCarDistance, queue_size=1)
    msg_distance = RaspiCarDistance()
    pub_wheel = rospy.Publisher('RaspiCarWheel', RaspiCarWheel, queue_size=1)
    msg_wheel = RaspiCarWheel()

    while not rospy.is_shutdown():
        try:
            line = serial_port.readline()
        except select.error as e:
            print(e)
        # print 'received:', line
        if line.startswith('#'):
            try:
                data = np.array(line[1:-1].split(','), dtype='|S4').astype(np.int64)
            except ValueError:
                continue

            time_stamp = int(time.time()*1000)

            msg_distance.info = '[mcu_bridge] ok.'
            msg_distance.time_stamp = time_stamp
            msg_distance.distance = data[0:3]

            msg_wheel.info = msg_distance.info
            msg_wheel.time_stamp = time_stamp
            msg_wheel.wheel_count = data[3:5]

            pub_distance.publish(msg_distance)
            pub_wheel.publish(msg_wheel)

            rospy.loginfo('%s, %d' % (msg_distance.info, time_stamp))

if __name__ == '__main__':
    try:
        serial_port = serial.Serial('/dev/ttyACM0', 115200)
        talker(serial_port)
    except rospy.ROSInterruptException:
        serial_port.close()
        pass
