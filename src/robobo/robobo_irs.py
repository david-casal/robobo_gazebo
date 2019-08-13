#!/usr/bin/env python

#This program is used to publish all infrared sensor values in one topic like in the real Robobo

# ROS libraries
import rospy

# ROS messages
from sensor_msgs.msg import Range
from robobo_msgs.msg import IRs


# Class that does all the work
class ROBOBO_IRS(object):

    # Initialize ROS publishers, subscribers and variables
    def __init__(self, robobo_name='robobo'):

        self.robobo_name = robobo_name
        self.pan_position = None
        self.tilt_position = None
        self.irs_dict = dict((key, None) for key in
                             ['front_c', 'front_l', 'front_ll', 'front_r', 'front_rr', 'back_c', 'back_l', 'back_r'])
        rospy.init_node('RoboboIRS', anonymous=True)
        # ROS publishers
        # IRs
        self.irs_pub = rospy.Publisher(str(self.robobo_name) + '/irs', IRs, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber(str(self.robobo_name) + '/front_c', Range, self.irs_cb, 'front_c')
        rospy.Subscriber(str(self.robobo_name) + '/front_l', Range, self.irs_cb, 'front_l')
        rospy.Subscriber(str(self.robobo_name) + '/front_ll', Range, self.irs_cb, 'front_ll')
        rospy.Subscriber(str(self.robobo_name) + '/front_r', Range, self.irs_cb, 'front_r')
        rospy.Subscriber(str(self.robobo_name) + '/front_rr', Range, self.irs_cb, 'front_rr')
        rospy.Subscriber(str(self.robobo_name) + '/back_c', Range, self.irs_cb, 'back_c')
        rospy.Subscriber(str(self.robobo_name) + '/back_l', Range, self.irs_cb, 'back_l')
        rospy.Subscriber(str(self.robobo_name) + '/back_r', Range, self.irs_cb, 'back_r')

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

    def run(self):
        rospy.spin()


def main():
    # Initializes and cleanup ROS node
    instance = ROBOBO_IRS()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print
        "Shutting down Robobo IRS"


if __name__ == '__main__':
    main()
