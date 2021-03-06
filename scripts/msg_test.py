#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ccn_raspicar_ros.msg import RaspiCarMsg
from ccn_raspicar_ros.msg import RaspiCarCamera
from ccn_raspicar_ros.msg import RaspiCarDistance
from ccn_raspicar_ros.msg import RaspiCarServo
from ccn_raspicar_ros.msg import RaspiCarWheel
from ccn_raspicar_ros.msg import RaspiCarWheelControl

def talker():
    rospy.init_node('RaspiCarCamera', anonymous=True)
    rate = rospy.Rate(2)
    pub = rospy.Publisher('RaspiCarCamera_signal', RaspiCarCamera, queue_size=1)
    msg = RaspiCarCamera()
    while not rospy.is_shutdown():
        msg.info = 'test test test'
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)
