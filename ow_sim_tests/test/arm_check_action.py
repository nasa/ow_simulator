#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import roslib
import unittest
from geometry_msgs.msg import Point
import ow_lander.msg
import argparse

from common_test_methods import test_action

# expected durations and final positions of each arm action
MAX_DURATION_UNSTOW          = 30.0
EXPECTED_FINAL_UNSTOW        = Point(1.796, -0.03159, -6.759)

MAX_DURATION_GUARDED         = 60.0
EXPECTED_FINAL_GUARDED       = Point(2.058, -0.1167, -0.1585)

MAX_DURATION_GRIND           = 80.0
EXPECTED_FINAL_GRIND         = Point(1.529, -0.2553, -6.750)

MAX_DURATION_DIG_CIRCULAR    = 60.0
EXPECTED_FINAL_DIG_CIRCULAR  = Point(2.335, 0.008707, -6.934)

MAX_DURATION_DISCARD         = 200.0
EXPECTED_FINAL_DISCARD       = Point(1.509, 0.9325, -6.748)

MAX_DURATION_DIG_LINEAR      = 110.0
EXPECTED_FINAL_DIG_LINEAR    = Point(2.364, -0.01750, -7.059)

MAX_DURATION_DELIVER         = 200.0
EXPECTED_FINAL_DELIVER       = Point(0.5597, -0.1448, -6.459)

MAX_DURATION_STOW            = 30.0
EXPECTED_FINAL_STOW          = Point(0.6004, -0.5449, -6.559)

DEFAULT_GROUND_HEIGHT  = -0.155

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)

class ArmCheckAction(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_check_action_test")

    cls.ignore_checks = '--ignore_checks' in sys.argv

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_unstow(self):
    unstow_result = test_action(self,
      'Unstow',
      ow_lander.msg.UnstowAction,
      ow_lander.msg.UnstowGoal(),
      MAX_DURATION_UNSTOW,
      expected_final = EXPECTED_FINAL_UNSTOW,
      expected_final_tolerance = 0.5, # unstow requires a really high tolerance
      server_timeout = 60.0,
      ignore_checks = self.ignore_checks
    )

  def test_02_guarded_move(self):
    guarded_move_result = test_action(self,
      'GuardedMove',
      ow_lander.msg.GuardedMoveAction,
      ow_lander.msg.GuardedMoveGoal(
        start = Point(2.0, 0.0, 0.3),
        normal = Point(0.0, 0.0, 1.0),
        search_distance = 0.5
      ),
      MAX_DURATION_GUARDED,
      expected_final = EXPECTED_FINAL_GUARDED,
      server_timeout = 15.0,
      ignore_checks = self.ignore_checks
    )

  def test_03_grind(self):
    grind_result = test_action(self,
      'Grind',
      ow_lander.msg.GrindAction,
      ow_lander.msg.GrindGoal(
        x_start = 1.65,
        y_start = 0.0,
        depth = 0.05,
        length = 0.6,
        parallel = True,
        ground_position = DEFAULT_GROUND_HEIGHT
      ),
      MAX_DURATION_GRIND,
      expected_final = EXPECTED_FINAL_GRIND,
      ignore_checks = self.ignore_checks
    )

  def test_04_dig_circular(self):
    dig_circular_result = test_action(self,
      'DigCircular',
      ow_lander.msg.DigCircularAction,
      ow_lander.msg.DigCircularGoal(
        x_start = 1.65,
        y_start = 0.0,
        depth = 0.01,
        parallel = True,
        ground_position = DEFAULT_GROUND_HEIGHT
      ),
      MAX_DURATION_DIG_CIRCULAR,
      expected_final = EXPECTED_FINAL_DIG_CIRCULAR,
      ignore_checks = self.ignore_checks
    )

  def test_05_discard(self):
    discard_result = test_action(self,
      'Discard',
      ow_lander.msg.DiscardAction,
      ow_lander.msg.DiscardGoal(
        discard = Point(1.5, 0.8, 0.65)
      ),
      MAX_DURATION_DISCARD,
      expected_final = EXPECTED_FINAL_DISCARD,
      ignore_checks = self.ignore_checks
  )

  def test_06_dig_linear(self):
    dig_linear_result = test_action(self,
      'DigLinear',
      ow_lander.msg.DigLinearAction,
      ow_lander.msg.DigLinearGoal(
        x_start = 1.46,
        y_start = 0.0,
        depth = 0.01,
        length = 0.1,
        ground_position = DEFAULT_GROUND_HEIGHT
      ),
      MAX_DURATION_DIG_LINEAR,
      expected_final = EXPECTED_FINAL_DIG_LINEAR,
      ignore_checks = self.ignore_checks
    )

  def test_07_deliver_sample(self):
    deliver_result = test_action(self,
      'Deliver',
      ow_lander.msg.DeliverAction,
      ow_lander.msg.DeliverGoal(),
      MAX_DURATION_DELIVER,
      expected_final = EXPECTED_FINAL_DELIVER,
      ignore_checks = self.ignore_checks
    )

  # def test_09_

  def test_08_stow(self):
    stow_result = test_action(self,
      'Stow',
      ow_lander.msg.StowAction,
      ow_lander.msg.StowGoal(),
      MAX_DURATION_STOW,
      expected_final = EXPECTED_FINAL_STOW,
      ignore_checks = self.ignore_checks
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'arm_check_action', ArmCheckAction)
