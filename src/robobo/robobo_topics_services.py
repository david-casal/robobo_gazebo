#!/usr/bin/env python

import math

# ROS libraries
import rospy

# ROS messages
from std_msgs.msg import Float64, Int32, Int16, Int8
from sensor_msgs.msg import Range
from robobo_msgs.msg import IRs
from robobo_msgs.srv import MoveWheels, MovePanTilt


# Class that does all the work
class ROBOBO_TOPICS_SERVICES(object):

    # Initialize ROS publishers, subscribers and variables
    def __init__(self, robobo_name='robobo'):

        self.robobo_name = robobo_name
        self.pan_position = None
        self.tilt_position = None
        self.irs_dict = dict((key, None) for key in
                             ['front_c', 'front_l', 'front_ll', 'front_r', 'front_rr', 'back_c', 'back_l', 'back_r'])
        rospy.init_node('RoboboTopicsServices', anonymous=True)
        # ROS publishers
        self.left_wheel_pub = rospy.Publisher(str(self.robobo_name) + '/left_wheel_motor', Float64, queue_size=1)
        self.right_wheel_pub = rospy.Publisher(str(self.robobo_name) + '/right_wheel_motor', Float64, queue_size=1)
        self.tilt_motor_pub = rospy.Publisher(str(self.robobo_name) + '/tilt_motor', Float64, queue_size=1)
        self.pan_motor_pub = rospy.Publisher(str(self.robobo_name) + '/pan_motor', Float64, queue_size=1)
        # IRs
        self.irs_pub = rospy.Publisher(str(self.robobo_name) + '/irs', IRs, queue_size=1)
        # Battery
        self.battery_base_pub = rospy.Publisher(str(self.robobo_name) + '/battery/base', Int8, queue_size=1)
        self.battery_phone_pub = rospy.Publisher(str(self.robobo_name) + '/battery/phone', Int8, queue_size=1)

        # ROS Subscribers
        self.tilt_pub = rospy.Subscriber(str(self.robobo_name) + '/tilt', Int16, self.tilt_cb)
        self.pan_pub = rospy.Subscriber(str(self.robobo_name) + '/pan', Int16, self.pan_cb)
        # IRs
        rospy.Subscriber(str(self.robobo_name) + '/front_c', Range, self.irs_cb, 'front_c')
        rospy.Subscriber(str(self.robobo_name) + '/front_l', Range, self.irs_cb, 'front_l')
        rospy.Subscriber(str(self.robobo_name) + '/front_ll', Range, self.irs_cb, 'front_ll')
        rospy.Subscriber(str(self.robobo_name) + '/front_r', Range, self.irs_cb, 'front_r')
        rospy.Subscriber(str(self.robobo_name) + '/front_rr', Range, self.irs_cb, 'front_rr')
        rospy.Subscriber(str(self.robobo_name) + '/back_c', Range, self.irs_cb, 'back_c')
        rospy.Subscriber(str(self.robobo_name) + '/back_l', Range, self.irs_cb, 'back_l')
        rospy.Subscriber(str(self.robobo_name) + '/back_r', Range, self.irs_cb, 'back_r')

        # ROS Services servers
        self.move_wheels_server = rospy.Service(str(self.robobo_name) + '/moveWheels', MoveWheels,
                                                self.handle_move_wheels)
        self.move_pan_tilt_server = rospy.Service(str(self.robobo_name) + '/movePanTilt', MovePanTilt,
                                                  self.handle_move_pan_tilt)

    def battery_pub(self):
        self.battery_base_pub.publish(Int8(100))
        self.battery_phone_pub.publish(Int8(100))

    def irs_cb(self, value, id):
        self.irs_dict[id] = value
        if not None in self.irs_dict.values():
            self.irs_pub.publish(
                self.irs_dict['front_c'],
                self.irs_dict['front_r'],
                self.irs_dict['front_rr'],
                self.irs_dict['front_l'],
                self.irs_dict['front_ll'],
                self.irs_dict['back_c'],
                self.irs_dict['back_r'],
                self.irs_dict['back_l']
            )
        # Publish battery level
        self.battery_pub()

    def pan_cb(self, pos):
        self.pan_position = pos.data

    def tilt_cb(self, pos):
        self.tilt_position = pos.data

    def handle_move_wheels(self, srv):
        end_time = rospy.get_time() + float(srv.time.data / 1000)
        # t = srv.time.data / 1000
        # if srv.lspeed.data < -100:
        #     lp = -100
        # elif srv.lspeed.data > 100:
        #     lp = 100
        # else:
        #     lp = srv.lspeed.data
        # if srv.rspeed.data < -100:
        #     rp = -100
        # elif srv.rspeed.data > 100:
        #     rp = 100
        # else:
        #     rp = srv.rspeed.data
        #
        # # Calculate left wheel Gazebo's target velocity
        # if lp < 0:
        #     lspeedGazebo = - math.radians((-4.625E-05 * abs(lp) ** 3 + 5.219E-03 * abs(lp) ** 2 + 6.357 * abs(lp) + 5.137E+01) + (
        #                 -3.253E-04 * abs(lp) ** 3 + 4.285E-02 * abs(lp) ** 2 + -2.064E+00 * abs(lp) - 1.770E+01)/t)
        # elif lp > 0:
        #     lspeedGazebo = math.radians((-4.625E-05 * lp ** 3 + 5.219E-03 * lp ** 2 + 6.357 * lp + 5.137E+01) + (
        #             -3.253E-04 * lp ** 3 + 4.285E-02 * lp ** 2 + -2.064E+00 * lp - 1.770E+01) / t)
        # else:
        #     lspeedGazebo = 0
        #
        # # Calculate right wheel Gazebo's target velocity
        # if rp < 0:
        #     rspeedGazebo = - math.radians(
        #         (-4.625E-05 * abs(rp) ** 3 + 5.219E-03 * abs(rp) ** 2 + 6.357 * abs(rp) + 5.137E+01) + (
        #                 -3.253E-04 * abs(rp) ** 3 + 4.285E-02 * abs(rp) ** 2 + -2.064E+00 * abs(
        #             rp) - 1.770E+01) / t)
        # elif rp > 0:
        #     rspeedGazebo = math.radians((-4.625E-05 * rp ** 3 + 5.219E-03 * rp ** 2 + 6.357 * rp + 5.137E+01) + (
        #             -3.253E-04 * rp ** 3 + 4.285E-02 * rp ** 2 + -2.064E+00 * rp - 1.770E+01) / t)
        # else:
        #     rspeedGazebo = 0

        while rospy.get_time() < end_time:
            self.left_wheel_pub.publish(Float64(srv.lspeed.data))
            self.right_wheel_pub.publish(Float64(srv.rspeed.data))
        self.left_wheel_pub.publish(Float64(0.0))
        self.right_wheel_pub.publish(Float64(0.0))
        return Int8(1)

    def handle_move_pan_tilt(self, srv):
        if 10 < srv.panPos.data < 345:
            if self.pan_position < srv.panPos.data:
                while self.pan_position < srv.panPos.data:
                    self.pan_motor_pub.publish(Float64(srv.panSpeed.data))
            else:
                while self.pan_position > srv.panPos.data:
                    self.pan_motor_pub.publish(Float64(-srv.panSpeed.data))
            self.pan_motor_pub.publish(Float64(0.0))
        # Falta set position en srv.panPos.data
        if 5 <= srv.tiltPos.data <= 110:
            if self.tilt_position < srv.tiltPos.data:
                while self.tilt_position < srv.tiltPos.data:
                    self.tilt_motor_pub.publish(Float64(srv.tiltSpeed.data))
            else:
                while self.tilt_position > srv.tiltPos.data:
                    self.tilt_motor_pub.publish(Float64(-srv.tiltSpeed.data))
            self.tilt_motor_pub.publish(Float64(0.0))
        return Int8(0)

    def run(self):
        rospy.spin()


def main():
    # Initializes and cleanup ROS node
    instance = ROBOBO_TOPICS_SERVICES()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print
        "Shutting down Robobo Topics and Services Server"


if __name__ == '__main__':
    main()
