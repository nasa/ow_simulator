#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math
import logging
from math import sin,cos,atan2,sqrt,fabs
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import CompressedImage
# Python libs
import sys, time
import numpy as np

VERBOSE=False

class power_grab:
    def __init__(self):
        
        self._logger = logging.getLogger('power_log')
        '''
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        self._logger.addHandler(out_hdlr)
        self._logger.setLevel(logging.INFO)
        '''
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.power_pub = rospy.Publisher('/total_power', Float64, queue_size = 10)

        # subscribed Topic

        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.callback, queue_size = 1)
        if VERBOSE :
            print "subscribed to /rrbot/joint_states"
            
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received data of type: "%s"' % ros_data    
            
        l = len(ros_data.name)

        power = 0
        for x in range(0,l):
            power = power + abs(ros_data.velocity[x]*ros_data.effort[x])
            
        
        # Publish total power
        self.power_pub.publish(power)
        logging.info('Hi from myfunc')
        self._logger.info(power)



def main(args):
    '''Initializes and cleanup ros node'''
    logger = logging.getLogger('power_log')
    logger.setLevel(logging.DEBUG)
    # create file handler which logs even debug messages
    fh = logging.FileHandler('power_data_OW_sf.csv')
    fh.setLevel(logging.DEBUG)
    # create console handler with a higher log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.ERROR)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s , %(name)s , %(levelname)s , %(message)s')
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)
    # add the handlers to logger
    logger.addHandler(ch)
    logger.addHandler(fh)
    #logging.info('Started')
   
   
    rospy.init_node('power_feature', anonymous=True)
    ic = power_grab()
    try:
        rospy.spin()
        logging.info('Hello from main')
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    #cv2.destroyAllWindows()
    logging.info('Finished')

if __name__ == '__main__':
    main(sys.argv)
