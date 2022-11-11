#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import roslib
import unittest
import ow_lander.msg
import argparse

from action_testing import *

DEFAULT_GROUND_HEIGHT  = -0.155

PKG = 'ow_sim_tests'
TEST_NAME = 'arm_check_action'
roslib.load_manifest(PKG)

class ArmCheckAction(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_check_action_test")

    # changes behavior of test_action and test_arm_action
    set_ignore_action_checks('--ignore_action_checks' in sys.argv)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_unstow(self):
    unstow_result = test_arm_action(self,
      'Unstow', ow_lander.msg.UnstowAction,
      ow_lander.msg.UnstowGoal(),
      TEST_NAME, "test_01_unstow",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_guarded_move(self):
    guarded_move_result = test_arm_action(self,
      'GuardedMove', ow_lander.msg.GuardedMoveAction,
      ow_lander.msg.GuardedMoveGoal(
        start = Point(2.0, 0.0, 0.3),
        normal = Point(0.0, 0.0, 1.0),
        search_distance = 0.5
      ),
      TEST_NAME, "test_02_guarded_move",
      server_timeout = 15.0
    )

  def test_03_grind(self):
    grind_result = test_arm_action(self,
      'Grind', ow_lander.msg.GrindAction,
      ow_lander.msg.GrindGoal(
        x_start = 1.65,
        y_start = 0.0,
        depth = 0.05,
        length = 0.6,
        parallel = True,
        ground_position = DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, "test_03_grind"
    )

  def test_04_dig_circular(self):
    dig_circular_result = test_arm_action(self,
      'DigCircular', ow_lander.msg.DigCircularAction,
      ow_lander.msg.DigCircularGoal(
        x_start = 1.65,
        y_start = 0.0,
        depth = 0.01,
        parallel = True,
        ground_position = DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, 'test_04_dig_circular'
    )

  def test_05_discard(self):
    discard_result = test_arm_action(self,
      'Discard', ow_lander.msg.DiscardAction,
      ow_lander.msg.DiscardGoal(
        discard = Point(1.5, 0.8, 0.65)
      ),
      TEST_NAME, 'test_05_discard'
    )

  def test_06_dig_linear(self):
    dig_linear_result = test_arm_action(self,
      'DigLinear', ow_lander.msg.DigLinearAction,
      ow_lander.msg.DigLinearGoal(
        x_start = 1.46,
        y_start = 0.0,
        depth = 0.01,
        length = 0.1,
        ground_position = DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, 'test_06_dig_linear'
    )

  def test_07_deliver_sample(self):
    deliver_result = test_arm_action(self,
      'Deliver', ow_lander.msg.DeliverAction,
      ow_lander.msg.DeliverGoal(),
      TEST_NAME, 'test_07_deliver_sample'
    )

  def test_08_stow(self):
    stow_result = test_arm_action(self,
      'Stow', ow_lander.msg.StowAction,
      ow_lander.msg.StowGoal(),
      TEST_NAME, 'test_08_stow'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'arm_check_action', ArmCheckAction)
