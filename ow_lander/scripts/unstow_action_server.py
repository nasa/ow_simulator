#! /usr/bin/env python

import rospy
import actionlib
import ow_lander.msg
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
import math
import constants
import threading
import utils
import activity_full_digging_traj
import random

from LanderInterface import MoveItInterface
from LanderInterface import JointStateSubscriber



class UnstowActionServer(object):
    
    _feedback = ow_lander.msg.UnstowFeedback()
    _result = ow_lander.msg.UnstowResult()
    
    def __init__(self,name):
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, ow_lander.msg.UnstowAction, execute_cb=self.on_unstow_action, auto_start = False)
        self._server.start()
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.UnstowFeedback()
        self._result = ow_lander.msg.UnstowResult()
        self._current_state = JointStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 5.0
        
    
    def _check_state(self):
        #rate = rospy.Rate(1) # 10hz
        #while not rospy.is_shutdown():
          self._xc = self._current_state.get_value()
          self._fdbk.current_x = self._xc
          self._fdbk.current_y = self._xc
          self._fdbk.current_z = self._xc
          self._result = self._fdbk
          self._server.publish_feedback(self._fdbk)
          #self._xc = (random.choice([1, 4, 8, 10, 3])) 
          #self._xc = (random.choice([1, 4, 8, 10, 3])) 
          #print (self._xc)
          print "state update loop"
        
    def _update_feedback(self):
        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
          self._fdbk.current_x = self._xc
          self._fdbk.current_y = self._xc
          self._fdbk.current_z = self._xc
          self._result = self._fdbk
          self._server.publish_feedback(self._fdbk)
          print "feedback loop"
          rate.sleep()
        
        
    def _update_motion(self):
        activity_full_digging_traj.unstow(self._interface.move_arm)
        
    def on_unstow_action(self,goal):
        
        # Record start time
        start_time = rospy.get_time()
        print "main loop"

        def now_from_start(start):
            return rospy.get_time() - start
        
        #r = rospy.Rate(10)
        success = True

        while ((now_from_start(start_time) < self._timeout)):# and not      rospy.is_shutdown()):
            # start executing the action
            if self._server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._server.set_preempted()
                success = False
                break
            
            self._check_state()
            #self._update_feedback()
            #self._update_mo-*tion()  
            
            #t1 = threading.Thread(target=self._check_state())
            #t2 = threading.Thread(target=self._update_feedback())
            t3 = threading.Thread(target=self._update_motion())
            #t1.setDaemon(True)
            #t2.setDaemon(True)
            #t3.setDaemon(True)
            #t2.start()
            #activity_full_digging_traj.unstow(self._interface.move_arm)
            #t1.start()
            t3.start()
            #t1.join()
            #t2.join()
            #t3.join()
            
        #for (k = 1:range(100)):
            #k = k+1
            
        if success:
            self._result.final_x = self._fdbk.current_x
            self._result.final_y = self._fdbk.current_y
            self._result.final_z = self._fdbk.current_z
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
    
if __name__ == '__main__':
    rospy.init_node('Unstow')
    server = UnstowActionServer(rospy.get_name())
    rospy.spin()
        
  
