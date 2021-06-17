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
    self._arm_move_group = moveit_commander.MoveGroupCommander(
        "arm", wait_for_servers=20.0)

  def _all_close(self, goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats
    @param: actual     A list of floats
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
      for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
          return False
    return True

  def _test_unstow(self):

  def test_unstow(self):

    rospy.init_node("test_unstow", anonymous=True)
    rospy.wait_for_service('/arm/unstow')
    success = False
    self._test_start_time = time.time()

    rospy.loginfo("unstow test started")
    time.sleep(15.0)

    # try:
    #   arm_unstow = rospy.ServiceProxy('/arm/unstow', Unstow)
    #   success = arm_unstow()
    # except rospy.ServiceException as e:
    #   print("arm/unstow threw an error: %s" % e)

    # self.assertTrue(success)

    rospy.loginfo("unstow test starting")

    joints_goal = self._arm_move_group.get_named_target_values("arm_unstowed")
    goal_state_achieved = False

    while not rospy.is_shutdown() and \
        (time.time() - self._test_start_time < self._test_duration) and \
        not goal_state_achieved:
      rospy.loginfo("testing in progress")
      time.sleep(0.1)
      joints_state = self._arm_move_group.get_current_joint_values()
      goal_state_achieved = self._all_close(joints_goal, joints_state, 0.01)

    self.assertTrue(goal_state_achieved)


if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'test_unstow', ArmCheck)
