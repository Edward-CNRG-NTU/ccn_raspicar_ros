#!/usr/bin/env python

import rospy
import numpy as np
import sys
import socket
import time
import struct
import RPi.GPIO as GPIO

from ccn_final.msg import RaspiCarWheel
from ccn_final.msg import RaspiCarWheelControl


class MotorControl(object):
    def __init__(self, control_pin=[16, 18, 11, 13], t=0.1, dc_level=80, balance=1.0, pwm_freq=500):
        self.control_pin = control_pin
        self.t = t
        self.balance = balance
        self.l_level = dc_level * 2 / (balance + 1)
        self.r_level = self.l_level * balance

        GPIO.setmode(GPIO.BOARD)
        [GPIO.setup(control_pin[pin], GPIO.OUT, initial=GPIO.LOW) for pin in range(4)]

        self.pwm_r1 = GPIO.PWM(control_pin[0], pwm_freq)
        self.pwm_r2 = GPIO.PWM(control_pin[1], pwm_freq)
        self.pwm_l1 = GPIO.PWM(control_pin[2], pwm_freq)
        self.pwm_l2 = GPIO.PWM(control_pin[3], pwm_freq)
        self.pwm_r1.start(0)
        self.pwm_r2.start(0)
        self.pwm_l1.start(0)
        self.pwm_l2.start(0)

    def stop(self):
        self.pwm_r1.ChangeDutyCycle(0)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(0)
        self.pwm_l2.ChangeDutyCycle(0)

    def forward(self, speed=1.0, t=None):
        self.pwm_r1.ChangeDutyCycle(self.r_level*speed)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(self.l_level*speed)
        self.pwm_l2.ChangeDutyCycle(0)
        if t is None:
            time.sleep(self.t)
        else:
            time.sleep(t)
        self.stop()

    def backward(self, speed=0.8, t=None):
        self.pwm_r1.ChangeDutyCycle(0)
        self.pwm_r2.ChangeDutyCycle(self.r_level*speed)
        self.pwm_l1.ChangeDutyCycle(0)
        self.pwm_l2.ChangeDutyCycle(self.l_level*speed)
        if t is None:
            time.sleep(self.t)
        else:
            time.sleep(t)
        self.stop()

    def turn_left(self, speed=0.6, t=None):
        self.pwm_r1.ChangeDutyCycle(self.r_level*speed)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(0)
        self.pwm_l2.ChangeDutyCycle(0)
        if t is None:
            time.sleep(self.t)
        else:
            time.sleep(t)
        self.stop()

    def turn_right(self, speed=0.6, t=None):
        self.pwm_r1.ChangeDutyCycle(0)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(self.l_level*speed)
        self.pwm_l2.ChangeDutyCycle(0)
        if t is None:
            time.sleep(self.t)
        else:
            time.sleep(t)
        self.stop()

    def arbitrary_speed(self, speed=[1.0, 1.0], t=None):
        if 0 < speed[0]:
            self.pwm_r1.ChangeDutyCycle(self.r_level * speed[0])
            self.pwm_r2.ChangeDutyCycle(0)
        elif speed[0] < 0:
            self.pwm_r1.ChangeDutyCycle(0)
            self.pwm_r2.ChangeDutyCycle(self.r_level * speed[0])
        if 0 < speed[1]:
            self.pwm_l1.ChangeDutyCycle(self.l_level * speed[1])
            self.pwm_l2.ChangeDutyCycle(0)
        elif speed[1] < 0:
            self.pwm_l1.ChangeDutyCycle(0)
            self.pwm_l2.ChangeDutyCycle(self.l_level * speed[1])
        if t is None:
            return
        else:
            time.sleep(t)
            self.stop()

    def cleanup(self):
        self.stop()
        self.pwm_r1.stop()
        self.pwm_r2.stop()
        self.pwm_l1.stop()
        self.pwm_l2.stop()
        GPIO.cleanup()


g_obstacle_detected = False
g_proximity = np.zeros([3])
g_wheel_count = np.zeros([2])


def turn_right_controlled(angle):
    wheel_last = g_wheel_count
    count = angle / 4.45
    while not rospy.is_shutdown():
        if not g_obstacle_detected:
            time.sleep(0.05)
            if g_wheel_count[0] - wheel_last[0] < count:
                motor.turn_right(speed=0.9, t=0.05)
            elif g_wheel_count[0] - wheel_last[0] > count:
                motor.turn_left(speed=0.8, t=0.03)
                break
            else:
                break
        else:
            time.sleep(0.1)


def turn_left_controlled(angle):
    wheel_last = g_wheel_count
    count = angle / 4.45
    while not rospy.is_shutdown():
        if not g_obstacle_detected:
            
            if g_wheel_count[1] - wheel_last[1] < count:
                motor.turn_left(speed=0.9, t=0.05)
            elif g_wheel_count[1] - wheel_last[1] > count:
                motor.turn_right(speed=0.8, t=0.03)
                break
            else:
                break

            time.sleep(0.05)
            
        else:
            time.sleep(0.1)


def forward_controlled(distance):
    wheel_last = g_wheel_count
    count = distance / 0.0113
    while not rospy.is_shutdown():        
        if not g_obstacle_detected:
            
            diff_of_both = g_wheel_count - wheel_last
            diff_between = diff_of_both[0] - diff_of_both[1]
            print(np.sum(diff_of_both)/2, diff_between)

            if np.sum(diff_of_both)/2 < count:
                motor.forward(speed=1.0, t=0.05)                
            else:
                break

            if diff_between > 0:
                motor.turn_left(speed=0.7, t=0.03 + np.abs(diff_between) * 0.005)
            elif diff_between < 0:
                motor.turn_right(speed=0.7, t=0.03 + np.abs(diff_between) * 0.005)
                
            time.sleep(0.05)
            
        else:
            time.sleep(0.1)


def callback_RaspiCarWheel(data):
    global g_wheel_count
    g_wheel_count = np.array(data.wheel_count)


def listener():
    rospy.Subscriber('RaspiCarWheel', RaspiCarWheel, callback_RaspiCarWheel)


def tcp_routine():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(30)
    try:
        server_address = ('', 23233)
        rospy.loginfo('[tcp_server] binding to %s port %s' % server_address)
        sock.bind(server_address)
        sock.listen(0)

        while not rospy.is_shutdown():

            try:
                rospy.loginfo('[tcp_server] waiting for new connection...')
                connection, client_address = sock.accept()
                rospy.loginfo('[tcp_server] accepted connection from %s %d.' % client_address)

                while not rospy.is_shutdown():
                    rospy.loginfo('[tcp_server] waiting for command...')
                    data = str(connection.recv(128))
                    rospy.loginfo('[tcp_server] received "%s".' % data)

                    if len(data) == 0:
                        rospy.loginfo('[tcp_server] connection closed.')
                        break
                    elif data.startswith('test'):
                        connection.sendall(b'ack test')
                        rospy.loginfo('[tcp_server] sending ack to the client.')
                    elif data.startswith('fwd'):
                        try:
                            value = float(data.split(':')[1])
                        except KeyError:
                            value = 0.1
                        except ValueError:
                            value = 0
                        forward_controlled(value)
                        connection.sendall(b'ack fwd')
                        rospy.loginfo('[tcp_server] sending ack to the client.')
                    elif data.startswith('right'):
                        try:
                            value = float(data.split(':')[1])
                        except KeyError:
                            value = 10
                        except ValueError:
                            value = 0
                        turn_right_controlled(value)
                        connection.sendall(b'ack right')
                        rospy.loginfo('[tcp_server] sending ack to the client.')
                    elif data.startswith('left'):
                        try:
                            value = float(data.split(':')[1])
                        except KeyError:
                            value = 10
                        except ValueError:
                            value = 0
                        turn_left_controlled(value)
                        connection.sendall(b'ack left')
                        rospy.loginfo('[tcp_server] sending ack to the client.')
                    # elif data.startswith('obstacle'):
                    #     global obstacle_detection_routine_stopper
                    #     try:
                    #         value = float(data.split(':')[1])
                    #     except KeyError:
                    #         if obstacle_detection_routine_stopper is None:
                    #             value = 1
                    #         else:
                    #             value = 0
                    #     except ValueError:
                    #         value = 0
                    #
                    #     if value > 0.0 and obstacle_detection_routine_stopper is None:
                    #         obstacle_detection_routine_stopper = launch_obstacle_detection_routine()
                    #     elif value == 0.0 and obstacle_detection_routine_stopper is not None:
                    #         obstacle_detection_routine_stopper.set()
                    #         obstacle_detection_routine_stopper = None
                    #
                    #     connection.sendall(b'ack')
                    #     rospy.loginfo('[tcp_server] sending ack to the client.')
                    else:
                        connection.sendall(b'error')
                        rospy.loginfo('[tcp_server] unexpected command.')

            except socket.timeout:
                rospy.loginfo('[tcp_server] sock.accept() time out.')

    except socket.error as e:
        rospy.loginfo(e)

    finally:
        sock.close()
        rospy.loginfo('[tcp_server] stopped.')


if __name__ == '__main__':
    motor = MotorControl(dc_level=70, t=0.3)

    try:
        rospy.init_node('RaspiCarTCPServer_node', anonymous=False)
        listener()
        tcp_routine()

    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)

    finally:
        motor.cleanup()

