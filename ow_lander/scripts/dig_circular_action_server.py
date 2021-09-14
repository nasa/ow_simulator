#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

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
from gazebo_msgs.msg import LinkStates
from moveit_commander.conversions import pose_to_list
import math
import constants
import utils
import activity_full_digging_traj
import action_deliver_sample
import action_dig_linear
import action_dig_circular
#from action_deliver_sample import deliver_sample

from LanderInterface import MoveItInterface
from LanderInterface import LinkStateSubscriber
from trajectory_async_execution import TrajectoryAsyncExecuter
from moveit_msgs.msg import RobotTrajectory



class DigCircularActionServer(object):
    
    def __init__(self,name):
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, 
                                                    ow_lander.msg.DigCircularAction, 
                                                    execute_cb=self.on_DigCircular_action, 
                                                    auto_start = False)
        self._server.start()
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.DigCircularFeedback()
        self._result = ow_lander.msg.DigCircularResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self.trajectory_async_executer = TrajectoryAsyncExecuter()
        self.trajectory_async_executer.connect("arm_controller")
        self.current_traj = RobotTrajectory()
        
        
    def switch_controllers(self, start_controller, stop_controller):
        rospy.wait_for_service('/controller_manager/switch_controller')
        success = False
        try:
          switch_controller = rospy.ServiceProxy(
          '/controller_manager/switch_controller', SwitchController)
          success = switch_controller(
          [start_controller], [stop_controller], 2, False, 1.0)
        except rospy.ServiceException as e:
          rospy.logerror("switch_controllers error: %s" % e)
        finally:
          # This sleep is a workaround for "start point deviates from current robot
          #  state" error on dig_circular trajectory execution.
          time.sleep(0.2)
          return success
        
    
    def _update_feedback(self):
 
        self._ls =  self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

        
        
    def _update_motion(self, goal):
        rospy.loginfo("DigCircular activity started")
        self.current_traj = None
        self.current_traj = action_dig_circular.dig_circular(self._interface.move_arm,
                                         self._interface.move_limbs, 
                                         self._interface.robot,self._interface.moveit_fk, goal)
        if self.current_traj == False: 
            return 
        else:
            n_points = len(self.current_traj.joint_trajectory.points)
            start_time =   self.current_traj.joint_trajectory.points[0].time_from_start
            end_time =      self.current_traj.joint_trajectory.points[n_points-1].time_from_start
            self._timeout =  (end_time -start_time)
        

        
    def on_DigCircular_action(self,goal):
        plan = self._update_motion(goal)
        if self.current_traj == False: 
            self._server.set_aborted(self._result)
            return 
        success = False

        self.trajectory_async_executer.execute(self.current_traj.joint_trajectory,
                                           done_cb=None,
                                           active_cb=None,
                                           feedback_cb=self.trajectory_async_executer.stop_arm_if_fault)

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout)):

           self._update_feedback()
           
        success = self.trajectory_async_executer.success() and self.trajectory_async_executer.wait()        
        
            
        if success:
            self._result.final.x = self._fdbk.current.x
            self._result.final.y = self._fdbk.current.y 
            self._result.final.z = self._fdbk.current.z 
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)

if __name__ == '__main__':
    rospy.init_node('DigCircular')
    server = DigCircularActionServer(rospy.get_name())
    rospy.spin()
        
  
