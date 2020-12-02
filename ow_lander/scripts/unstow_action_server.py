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
from moveit_commander.conversions import pose_to_list
import math
import constants

def unstow_move(move_arm): 
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type args: List[bool, bool, float, float, float, float, float, float, float]
  """
  target_x=2.0
  target_y=0.0
  target_z=0.3
  direction_x=0.0
  direction_y=0.0
  direction_z=1.0
  search_distance = 0.5

  targ_x = target_x
  targ_y = target_y
  targ_z = target_z

  # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
  targ_elevation = -0.2
  if (targ_z+targ_elevation)==0:
    offset = search_distance
  else:
    offset = (targ_z*search_distance)/(targ_z+targ_elevation)

  # Compute shoulder yaw angle to target
  alpha = math.atan2( (targ_y+direction_y*offset)-constants.Y_SHOU, (targ_x+direction_x*offset)-constants.X_SHOU)
  h = math.sqrt(pow( (targ_y+direction_y*offset)-constants.Y_SHOU,2) + pow( (targ_x+direction_x*offset)-constants.X_SHOU,2) )
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin (l/h)

  # Move to pre move position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 0
  joint_goal[constants.J_HAND_YAW] = 0
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal  [constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta

  # If out of joint range, abort
  #if (is_shou_yaw_goal_in_range(joint_goal) == False):
  #   return False

  joint_goal[constants.J_SCOOP_YAW] = 0
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()

  print "Done planning approach of guarded_move"

  return True

class LanderInteface(object):
  
  def __init__(self):
    super(LanderInteface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_arm = moveit_commander.MoveGroupCommander("arm")
    move_limbs = moveit_commander.MoveGroupCommander("limbs")
    move_grinder = moveit_commander.MoveGroupCommander("grinder")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.move_arm = move_arm
    self.move_limbs = move_limbs
    self.move_grinder = move_grinder


class UnstowActionServer(object):
    
    #interface = LanderInteface()
    
    #activity_guarded_move.pre_guarded_move(interface.move_arm)
    _feedback = ow_lander.msg.UnstowFeedback()
    _result = ow_lander.msg.UnstowResult()
    
    def __init__(self,name):
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, ow_lander.msg.UnstowAction, execute_cb=self.on_unstow_action, auto_start = False)
        self._server.start()
        
    def on_unstow_action(self,goal):
        
        r = rospy.Rate(1)
        success = True
                # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        interface = LanderInteface()
        unstow_move(interface.move_arm)
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._server.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._server.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
    
if __name__ == '__main__':
    rospy.init_node('Unstow')
    server = UnstowActionServer(rospy.get_name())
    rospy.spin()
        
  
