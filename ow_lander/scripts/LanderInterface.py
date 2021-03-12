#!/usr/bin/env python

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
from moveit_commander.conversions import pose_to_list
from gazebo_msgs.msg import LinkStates
import math
import constants
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import tf2_ros
from moveit_msgs.srv import GetPositionFK


class MoveItInterface(object):
  
  def __init__(self):
    """ Initialize robot commander"""  
    super(MoveItInterface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_arm = moveit_commander.MoveGroupCommander("arm")
    move_limbs = moveit_commander.MoveGroupCommander("limbs")
    move_grinder = moveit_commander.MoveGroupCommander("grinder")
    self.move_arm = move_arm
    self.move_limbs = move_limbs
    self.move_grinder = move_grinder
    self.robot = robot
    self._buffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._buffer)
    ### forward kinematics: enable forward kinematics service from moveit
    rospy.wait_for_service('compute_fk')
    try:
      self.moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    except rospy.ServiceException, e:
      rospy.logerror("Service call failed: %s"%e)
    
    
class JointStateSubscriber:
  def __init__(self):
    """Initialize joint subscriber"""
    self.subscriber = rospy.Subscriber('/joint_states', JointState, self.callback, queue_size = 1)
    self.history = []
    rate = rospy.Rate(100)    
    
  def callback(self, ros_data): 
    """
    :type ros_data: class 'sensor_msgs.msg._JointState.JointState'
    """
    #self._value = len(ros_data.name)
    #num_joints = len(ros_data.name)        
    #self._state_value = ros_data.position[num_joints-1]   
    self._value = len(ros_data.name)
    num_joints = len(ros_data.name)
    power = 0
    for x in range(0,num_joints):
        #power = power + abs(ros_data.velocity[x]*ros_data.effort[x])
        power = 0
        # Publish total power
    self._state_value = power    
    #self.power_pub.publish(power)

    
  def get_value(self):
      return self._state_value
  
class LinkStateSubscriber:
    
  def __init__(self):
    #self.subscriber = rospy.Subscriber("/gazebo/link_states", LinkStates, self._handle_link_states)
    self._link_value = None
    self.subscriber = rospy.Subscriber("/gazebo/link_states", LinkStates, self._handle_link_states)
    
  def _handle_link_states(self, data):
    """
    :type data: gazebo_msgs.msg._LinkStates.LinkStates
    """
    #idx = data.name.index("lander::l_scoop_tip")
    try:
     idx = data.name.index("lander::l_scoop")
    except ValueError:
      rospy.logerr_throttle(
          1, "GroundDetector: lander::l_scoop_tip not found in link_states")
      return
    self._link_value = (data.pose[idx].position)
  