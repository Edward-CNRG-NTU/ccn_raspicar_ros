#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import time

from ccn_final.msg import RaspiCarCamera

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]

def talker(camera):
    rospy.init_node('RaspiCarCamera_node', anonymous=False)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('RaspiCarCamera', RaspiCarCamera, queue_size=1)
    msg = RaspiCarCamera()
    while not rospy.is_shutdown():
        (_, frame) = camera.read()
        (_, frame) = camera.read()
        msg.info = '[camera] ok.'
        msg.time_stamp = int(time.time()*1000)
        result, imgencode = cv2.imencode('.jpg', frame, encode_param)
        msg.camera_jpg = imgencode.tobytes()
        pub.publish(msg)
        # rospy.loginfo('%s, %d' % (msg.info, msg.time_stamp))
        print('%s, %d' % (msg.info, msg.time_stamp))

        rate.sleep()

if __name__ == '__main__':
    Frame_Width = 320
    Frame_Height = 240
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, Frame_Width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, Frame_Height)

    try:
        talker(camera)

    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)

    finally:
        camera.release()

