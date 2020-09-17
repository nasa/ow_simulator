#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math
from math import sin,cos,atan2,sqrt,fabs
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import CompressedImage
# Python libs
import sys, time
import numpy as np


VERBOSE=True

class MechanicalPower:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.power_pub = rospy.Publisher('/mechanical_power', Float64, queue_size = 10)
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.callback, queue_size = 1)
        rate = rospy.Rate(100)    
            

    def callback(self, ros_data):
        self._value = len(ros_data.name)
        l = len(ros_data.name)
        power = 0
        for x in range(0,l):
            power = power + abs(ros_data.velocity[x]*ros_data.effort[x])

        # Publish total power
        self.power_pub.publish(power)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('mechanical_power_arm', anonymous=True)
    mp = MechanicalPower()
    try:
        #ic = ScoopForce()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down scoop force module"


if __name__ == '__main__':
    main(sys.argv)
 
