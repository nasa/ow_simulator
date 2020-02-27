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
        
        self._logger = logging.getLogger('link_log')
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
        self._power = 0
        self._l_scoop_effort = 0
        self._l_scoop_velocity = 0
        self.j_ant_pan_e = 0 
        self.j_ant_pan_v = 0
        self.j_ant_tilt_e= 0 
        self.j_ant_tilt_v= 0 
        self.j_dist_pitch_e = 0
        self.j_dist_pitch_v = 0
        self.j_hand_yaw_e = 0
        self.j_hand_yaw_v = 0 
        self.j_prox_pitch_e = 0 
        self.j_prox_pitch_v = 0 
        self.j_scoop_yaw_e = 0
        self.j_scoop_yaw_v = 0
        self.j_shou_pitch_e= 0
        self.j_shou_pitch_v= 0
        self.j_shou_yaw_e = 0
        self.j_shou_yaw_v = 0
 
        # subscribed Topic

        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.callback, queue_size = 1)
        self.link_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_callback, queue_size = 1)
        if VERBOSE :
            print "subscribed to /rrbot/joint_states"
            
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        '''
        name: [j_ant_pan, j_ant_tilt, j_dist_pitch, j_hand_yaw, j_prox_pitch, j_scoop_yaw, j_shou_pitch,
        j_shou_yaw]
        '''

        if VERBOSE :
            #print 'received data of type: "%s"' % ros_data    
            print 'callback 1 '
        l = len(ros_data.name)

        power = 0
        for x in range(0,l):
            power = power + abs(ros_data.velocity[x]*ros_data.effort[x])
            
        
        # Publish total power
        self.power_pub.publish(power)
        self._power = power
        self._l_scoop_effort =  ros_data.effort[5]
        self._l_scoop_velocity = ros_data.velocity[5]
        self.j_ant_pan_v = ros_data.velocity[0]
        self.j_ant_pan_e = ros_data.effort[0]
        self.j_ant_tilt_v =  ros_data.velocity[1]
        self.j_ant_tilt_e =  ros_data.effort[1]
        self.j_dist_pitch_v =  ros_data.velocity[2]
        self.j_dist_pitch_e =  ros_data.effort[2]
        self.j_hand_yaw_v =  ros_data.velocity[3]
        self.j_hand_yaw_e =  ros_data.effort[3]
        self.j_prox_pitch_v =  ros_data.velocity[4]
        self.j_prox_pitch_e =  ros_data.effort[4]
        self.j_scoop_yaw_v =  ros_data.velocity[5]
        self.j_scoop_yaw_e =  ros_data.effort[5]
        self.j_shou_pitch_v =  ros_data.velocity[6]
        self.j_shou_pitch_e =  ros_data.effort[6]
        self.j_shou_yaw_v =  ros_data.velocity[7]
        self.j_shou_yaw_e =  ros_data.effort[7]
        logging.info('Hi from myfunc')
        #self._logger.info(power)
        #self._logger.info(ros_data.velocity[0])
        #self._logger.info(ros_data.effort[0])
        
    def link_callback(self, ros_data):
        if VERBOSE :
            #print 'received data of type: "%s"' % ros_data    
            print 'callback 2 '
        '''
        name: ['ow_sun::ellipsoid', 'ow_jupiter::ellipsoid', 'ow_light_probe::sphere', 'heightmap::link',
        'gui_camera_sim::link', 'lander::base_link', 'lander::l_ant_foot', 'lander::l_ant_panel',
        'lander::lander_lights_link', 'lander::l_shou', 'lander::l_prox', 'lander::l_dist',
        'lander::l_wrist', 'lander::l_hand', 'lander::l_scoop']
        '''
        l = len(ros_data.name)
        #print (l)
        link_pose = ros_data.pose[14]
        #self._logger.info(link_pose.position.z)
        #self._logger.info('Current pos z %f and x = %f m/s' % (link_pose.position.z, link_pose.position.x))
        #self._logger.info('%f , %f' % (link_pose.position.z, link_pose.position.x))
        #self._logger.info('%f , %f, %f, %f' % (link_pose.position.z, self._power, self._l_scoop_effort, self._l_scoop_velocity)) #z, total power, torque, velocty 
        self._logger.info('%f , %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f' % (link_pose.position.z, self._power,         
        self.j_ant_pan_v,
        self.j_ant_pan_e ,
        self.j_ant_tilt_v ,
        self.j_ant_tilt_e ,
        self.j_dist_pitch_v ,
        self.j_dist_pitch_e ,
        self.j_hand_yaw_v ,
        self.j_hand_yaw_e ,
        self.j_prox_pitch_v ,
        self.j_prox_pitch_e ,
        self.j_scoop_yaw_v, 
        self.j_scoop_yaw_e, 
        self.j_shou_pitch_v, 
        self.j_shou_pitch_e, 
        self.j_shou_yaw_v, 
        self.j_shou_yaw_e)) #z, total power, torque, velocty 
        #self._logger.info(10)
        '''
        power = 0
        for x in range(l,l):
            #power = power + abs(ros_data.velocity[x]*ros_data.effort[x])
            link_pose = ros_data.pose[x]
            self._logger.info(link_pose.position.z)

            link_quaternion = link_pose.orientation
            q = [link_quaternion.x,link_quaternion.y,link_quaternion.z,link_quaternion.w] #creates list from quaternion since it was not originally
            link_orientation = euler_from_quaternion(q) #transfrom from quaternion to euler angles
            end_effector_x = link_pose.position.x +sin(link_orientation[1])
            end_effector_y = link_pose.position.y
            end_effector_z = link_pose.position.z -cos(link_orientation[1])
            end_effector_position = [end_effector_x,end_effector_y,end_effector_z]
            #print(end_effector_position) 
            angles  = np.asarray(link_orientation)
            ypr =  180/3.14*angles
            #print(ypr)
            pitch  = ypr[2]
            end  =  end_effector_z + cos(link_orientation[2])
            #print (end)
         '''
def main(args):
    '''Initializes and cleanup ros node'''
    logger = logging.getLogger('link_log')
    logger.setLevel(logging.DEBUG)
    # create file handler which logs even debug messages
    fh = logging.FileHandler('link_data_OW_scoop_z_power_torque_vel_0_test.csv')
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
   
   
    rospy.init_node('link_feature', anonymous=True)
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
