#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import roslib
import unittest
import ow_lander.msg
import owl_msgs.msg
import argparse

from action_testing import *
from ow_lander import constants

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

  def test_01_arm_unstow(self):
    arm_unstow_result = test_arm_action(self,
      'ArmUnstow', owl_msgs.msg.ArmUnstowAction,
      owl_msgs.msg.ArmUnstowGoal(),
      TEST_NAME, "test_01_arm_unstow",
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

  def test_03_task_grind(self):
    task_grind_result = test_arm_action(self,
      'TaskGrind', owl_msgs.msg.TaskGrindAction,
      owl_msgs.msg.TaskGrindGoal(
        x_start = 1.65,
        y_start = 0.0,
        depth = 0.05,
        length = 0.6,
        parallel = True,
        ground_position = constants.DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, "test_03_task_grind"
    )

  def test_04_task_scoop_circular(self):
    task_scoop_circular_result = test_arm_action(self,
      'TaskScoopCircular', owl_msgs.msg.TaskScoopCircularAction,
      owl_msgs.msg.TaskScoopCircularGoal(
        frame = 0,
        relative =False,
        point = Point(1.65, 0.0, constants.DEFAULT_GROUND_HEIGHT),
        depth = 0.1,
        parallel = True
      ),
      TEST_NAME, 'test_04_task_scoop_circular'
    )

  def test_05_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 0,
        relative = False,
        point = Point(1.5, 0.8, constants.DEFAULT_GROUND_HEIGHT),
        height = 0.7
      ),
      TEST_NAME, 'test_05_task_discard_sample'
    )

  def test_06_task_scoop_linear(self):
    task_scoop_linear_result = test_arm_action(self,
      'TaskScoopLinear', owl_msgs.msg.TaskScoopLinearAction,
      owl_msgs.msg.TaskScoopLinearGoal(
        frame = 0,
        relative = False,
        point = Point(1.46, 0.0, constants.DEFAULT_GROUND_HEIGHT),
        depth = 0.01,
        length = 0.1
      ),
      TEST_NAME, 'test_06_task_scoop_linear'
    )

  def test_07_task_deliver_sample(self):
    task_deliver_sample_result = test_arm_action(self,
      'TaskDeliverSample', owl_msgs.msg.TaskDeliverSampleAction,
      owl_msgs.msg.TaskDeliverSampleGoal(),
      TEST_NAME, 'test_07_task_deliver_sample'
    )

  def test_08_arm_stow(self):
    arm_stow_result = test_arm_action(self,
      'ArmStow', owl_msgs.msg.ArmStowAction,
      owl_msgs.msg.ArmStowGoal(),
      TEST_NAME, 'test_08_arm_stow'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'arm_check_action', ArmCheckAction)
