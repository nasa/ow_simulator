#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import roslib
import unittest
import time
from ow_lander.srv import *
import moveit_commander

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)

# a class that monitors minimum reported frame rate by a gazebo simulation
class ArmCheck(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)

    self._test_duration = rospy.get_param("/arm_check/test_duration")
    moveit_commander.roscpp_initialize(sys.argv)
    self._robot = moveit_commander.RobotCommander()
    self._joint_names = self._robot.get_joint_names("arm")
    self._arm_move_group = moveit_commander.MoveGroupCommander(
        "arm", wait_for_servers=20.0)

  def _map_named_joint_target_to_list(self, joints_target):
    return [joints_target[n] for n in self._joint_names]

  def _normalize_angle(self, angle):
    """
    Convenience method to nomrlaize angles to the range [-pi, pi]
    @param angle: An angle that might need to be normalized
    @returns: the normalized value of the angle
    """
    from math import pi
    angle =  angle % (2*pi)
    angle = (angle + 2*pi) % (2*pi)
    if angle > pi:
        angle -= 2*pi
    return angle

  def _all_close(self, goal, actual, tolerance=0.01):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats
    @param: actual     A list of floats
    @param: tolerance  A float
    @returns: bool
    """
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False
    return True

  def _test_unstow(self):
    rospy.init_node("test_unstow", anonymous=True)
    rospy.wait_for_service('/arm/unstow')
    success = False
    self._test_start_time = time.time()

    try:
      arm_unstow = rospy.ServiceProxy('/arm/unstow', Unstow)
      success = arm_unstow()
    except rospy.ServiceException as e:
      rospy.logerr("arm/unstow threw an error: %s" % e)

    self.assertTrue(success)

    joints_target = self._arm_move_group.get_named_target_values("arm_unstowed")
    goal_state_achieved = False

    joints_goal = self._map_named_joint_target_to_list(joints_target)

    while not rospy.is_shutdown() and \
        (time.time() - self._test_start_time < self._test_duration) and \
        not goal_state_achieved:
      time.sleep(0.2)

      joints_state = self._arm_move_group.get_current_joint_values()
      rospy.loginfo("joints_goal = {}".format(joints_goal))
      normalized_joints_state = [ self._normalize_angle(e) for e in joints_state ]
      formatted_joints_state = [ '%.2f' % e for e in normalized_joints_state ]
      rospy.loginfo("joints_state = {}".format(formatted_joints_state))
      goal_state_achieved = self._all_close(joints_goal, normalized_joints_state)
      rospy.loginfo("achieved? {}".format(goal_state_achieved))

    self.assertTrue(goal_state_achieved)

  def test_unstow(self):
    try:
      self._test_unstow()
    except Exception as e:
      import traceback
      rospy.logerr("{}".format(traceback.format_exc()))
      rospy.logerr("exception: {}".format(e))
    except:
      rospy.logerr("Unexpected error: {}".format(sys.exc_info()[0]))
      raise


if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'test_unstow', ArmCheck)
