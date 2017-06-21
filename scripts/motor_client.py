#!/usr/bin/env python

import time
import rospy

from ccn_raspicar_ros.srv import RaspiCarMotorControl


if __name__ == '__main__':
    command = 'fwd:0.05'
    print(command)

    try:
        rospy.wait_for_service('RaspiCarMotorControl')

        try:
            proxy = rospy.ServiceProxy('RaspiCarMotorControl', RaspiCarMotorControl)
            t = time.time()
            response = proxy(command, 'ok')
            print(response, time.time()-t)
        except rospy.ServiceException as e:
            print(e)

    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)

    finally:
        pass
