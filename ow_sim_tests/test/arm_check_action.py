#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import roslib
import unittest
import moveit_commander
import actionlib
from geometry_msgs.msg import Point
import ow_lander.msg
from math import sqrt

from common_test_methods import test_action

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)

# expected durations and final positions of each arm action ran with defaults
UNSTOW_MAX_DURATION = 30.0
UNSTOW_EXPECTED_FINAL = Point(1.796, -0.03159, -6.759)

GUARDED_MAX_DURATION = 60.0
GUARDED_EXPECTED_FINAL = Point(2.058, -0.1167, -0.1585)

GRIND_MAX_DURATION = 80.0
GRIND_EXPECTED_FINAL = Point(1.529, -0.2553, -6.750)

DIG_CIRCULAR_MAX_DURATION = 60.0
DIG_CIRCULAR_EXPECTED_FINAL = Point(2.335, 0.008707, -6.934)

DISCARD_MAX_DURATION = 200.0
DISCARD_EXPECTED_FINAL = Point(1.509, 0.9325, -6.748)

DIG_LINEAR_MAX_DURATION = 110.0
DIG_LINEAR_EXPECTED_FINAL = Point(2.364, -0.01750, -7.059)

DELIVER_MAX_DURATION = 200.0
DELIVER_EXPECTED_FINAL = Point(0.5597, -0.1448, -6.459)

STOW_MAX_DURATION = 30.0
STOW_EXPECTED_FINAL = Point(0.6004, -0.5449, -6.559)

DEFAULT_GROUND_HEIGHT = -0.155

class ArmCheckAction(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node("arm_check_action_test", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    self._robot = moveit_commander.RobotCommander()
    self._joint_names = self._robot.get_joint_names("arm")
    self._arm_move_group = moveit_commander.MoveGroupCommander(
        "arm", wait_for_servers=20.0)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_unstow(self):
    unstow_result = test_action(self,
      'Unstow',
      ow_lander.msg.UnstowAction,
      ow_lander.msg.UnstowGoal(),
      UNSTOW_MAX_DURATION,
      expected_final = UNSTOW_EXPECTED_FINAL,
      expected_final_tolerance = 0.5, # unstow requires a really high tolerance
      server_timeout = 25.0
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
      GUARDED_MAX_DURATION,
      expected_final = GUARDED_EXPECTED_FINAL,
      server_timeout = 15.0
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
      GRIND_MAX_DURATION,
      expected_final = GRIND_EXPECTED_FINAL
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
      DIG_CIRCULAR_MAX_DURATION,
      expected_final = DIG_CIRCULAR_EXPECTED_FINAL
    )

  def test_05_discard(self):
    discard_result = test_action(self,
      'Discard',
      ow_lander.msg.DiscardAction,
      ow_lander.msg.DiscardGoal(
        discard = Point(1.5, 0.8, 0.65)
      ),
      DISCARD_MAX_DURATION,
      expected_final = DISCARD_EXPECTED_FINAL
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
      DIG_LINEAR_MAX_DURATION,
      expected_final = DIG_LINEAR_EXPECTED_FINAL
    )

  def test_07_deliver_sample(self):
    deliver_result = test_action(self,
      'Deliver',
      ow_lander.msg.DeliverAction,
      ow_lander.msg.DeliverGoal(),
      DELIVER_MAX_DURATION,
      expected_final = DELIVER_EXPECTED_FINAL
    )

  # def test_09_

  def test_08_stow(self):
    stow_result = test_action(self,
      'Stow',
      ow_lander.msg.StowAction,
      ow_lander.msg.StowGoal(),
      STOW_MAX_DURATION,
      expected_final = STOW_EXPECTED_FINAL
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'arm_check_action', ArmCheckAction)
