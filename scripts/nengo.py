#!/usr/bin/env python

import rospy
import numpy as np
import nengo

from ccn_final.msg import RaspiCarDistance
from ccn_final.msg import RaspiCarWheel
from ccn_final.msg import RaspiCarServo


def callback_RaspiCarDistance(data):
    rospy.loginfo('%s, %d, distance = [%3d,%3d,%3d]' % (data.info, data.time_stamp,
                                                        data.distance[0], data.distance[1], data.distance[2]))


def callback_RaspiCarWheel(data):
    rospy.loginfo('%s, %d, wheel_count = [%6d,%6d]' % (data.info, data.time_stamp,
                                                       data.wheel_count[0], data.wheel_count[1]))


def callback_RaspiCarServo(data):
    rospy.loginfo('%s, %d, servo_control = [%6d]' % (data.info, data.time_stamp, data.servo_control))


def listener():
    rospy.init_node('RaspiCarViewTopics_node', anonymous=True)
    rospy.Subscriber('RaspiCarDistance', RaspiCarDistance, callback_RaspiCarDistance)
    rospy.Subscriber('RaspiCarWheel', RaspiCarWheel, callback_RaspiCarWheel)
    rospy.Subscriber('RaspiCarServo', RaspiCarServo, callback_RaspiCarServo)
    rospy.spin()


if __name__ == '__main__':
    try:        
        print(nengo.__version__)
        listener()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)
        pass

