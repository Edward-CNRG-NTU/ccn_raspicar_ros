#!/usr/bin/env python

import rospy
import time

from ccn_final.msg import RaspiCarServo


def talker():
    rospy.init_node('RaspiCarServo_node', anonymous=False)
    rate = rospy.Rate(5)
    pub = rospy.Publisher('RaspiCarServo', RaspiCarServo, queue_size=10)
    msg = RaspiCarServo()
    while not rospy.is_shutdown():
        msg.info = '[servo_control] ok.'
        msg.time_stamp = int(time.time()*1000)
        msg.servo_control = int(time.time() % 30)
        pub.publish(msg)
        rospy.loginfo('%s, %d' % (msg.info, msg.time_stamp))

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
