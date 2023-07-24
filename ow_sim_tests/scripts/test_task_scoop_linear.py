#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest
import owl_msgs.msg

from action_testing import *
from ow_lander import constants

PKG = 'ow_sim_tests'
TEST_NAME = 'task_scoop_linear'
roslib.load_manifest(PKG)

class TestTaskScoopLinear(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_task_scoop_linear")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_task_scoop_linear(self):
    task_scoop_linear_result = test_arm_action(self,
      'TaskScoopLinear', owl_msgs.msg.TaskScoopLinearAction,
      owl_msgs.msg.TaskScoopLinearGoal(
        frame = 0,
        relative = False,
        point = Point(1.1, 0.0, 0.1),
        depth = 0.01,
        length = 0.5
      ),
      TEST_NAME, "test_01_task_scoop_linear",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_task_grind(self):
    task_grind_result = test_arm_action(self,
      'TaskGrind', owl_msgs.msg.TaskGrindAction,
      owl_msgs.msg.TaskGrindGoal(
        x_start = 1.6,
        y_start = 0.0,
        depth = 0.15,
        length = 0.8,
        parallel = True,
        ground_position = constants.DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, "test_02_task_grind",
    )
  
  def test_03_task_scoop_linear(self):
    task_scoop_linear_result = test_arm_action(self,
      'TaskScoopLinear', owl_msgs.msg.TaskScoopLinearAction,
      owl_msgs.msg.TaskScoopLinearGoal(
        frame = 0,
        relative = False,
        point = Point(1.4, 0.0, constants.DEFAULT_GROUND_HEIGHT),
        depth = 0.01,
        length = 0.3
      ),
      TEST_NAME, "test_03_task_scoop_linear"
    )

  def test_04_task_scoop_linear(self):
    task_scoop_linear_result = test_arm_action(self,
      'TaskScoopLinear', owl_msgs.msg.TaskScoopLinearAction,
      owl_msgs.msg.TaskScoopLinearGoal(
        frame = 0,
        relative = True,
        point = Point(-1.081, -0.01, -0.52),
        depth = 0.02,
        length = 0.2
      ),
      TEST_NAME, "test_04_task_scoop_linear"
    )

  def test_05_task_scoop_linear(self):
    task_scoop_linear_result = test_arm_action(self,
      'TaskScoopLinear', owl_msgs.msg.TaskScoopLinearAction,
      owl_msgs.msg.TaskScoopLinearGoal(
        frame = 1,
        relative = True,
        point = Point(-0.9, -0.05, -0.5),
        depth = 0.01,
        length = 0.05
      ),
      TEST_NAME, "test_05_task_scoop_linear"
    )

  def test_06_task_grind(self):
    task_grind_result = test_arm_action(self,
      'TaskGrind', owl_msgs.msg.TaskGrindAction,
      owl_msgs.msg.TaskGrindGoal(
        x_start = 1.6,
        y_start = 0.4,
        depth = 0.05,
        length = 0.8,
        parallel = True,
        ground_position = constants.DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, "test_06_task_grind"
    )

  def test_07_task_scoop_linear(self):
    task_scoop_linear_result = test_arm_action(self,
      'TaskScoopLinear', owl_msgs.msg.TaskScoopLinearAction,
      owl_msgs.msg.TaskScoopLinearGoal(
        frame = 0,
        relative = False,
        point = Point(1.60, 0.4, constants.DEFAULT_GROUND_HEIGHT),
        depth = 0.01,
        length = 0.1
      ),
      TEST_NAME, "test_07_task_scoop_linear"
    )

  def test_08_task_grind(self):
    task_grind_result = test_arm_action(self,
      'TaskGrind', owl_msgs.msg.TaskGrindAction,
      owl_msgs.msg.TaskGrindGoal(
        x_start = 1.6,
        y_start = -0.2,
        depth = 0.05,
        length = 0.7,
        parallel = True,
        ground_position = constants.DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, "test_08_task_grind"
    )

  def test_09_task_scoop_linear(self):
    task_scoop_linear_result = test_arm_action(self,
      'TaskScoopLinear', owl_msgs.msg.TaskScoopLinearAction,
      owl_msgs.msg.TaskScoopLinearGoal(
        frame = 0,
        relative = False,
        point = Point(1.60, -0.2, constants.DEFAULT_GROUND_HEIGHT),
        depth = 0.01,
        length = 0.03
      ),
      TEST_NAME, "test_09_task_scoop_linear"
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestTaskScoopLinear)
