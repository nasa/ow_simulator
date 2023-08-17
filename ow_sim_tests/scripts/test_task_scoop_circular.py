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
TEST_NAME = 'task_scoop_circular'
roslib.load_manifest(PKG)

class TestTaskScoopCircular(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_task_scoop_circular")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_task_scoop_circular(self):
    task_scoop_circular_result = test_arm_action(self,
      'TaskScoopCircular', owl_msgs.msg.TaskScoopCircularAction,
      owl_msgs.msg.TaskScoopCircularGoal(
        frame = 0,
        relative = False,
        point = Point(1.65, 0.0, constants.DEFAULT_GROUND_HEIGHT + 0.5),
        depth = 0.01,
        parallel = True
      ),
      TEST_NAME, "test_01_task_scoop_circular",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_task_grind(self):
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
      TEST_NAME, "test_02_task_grind",
    )
  
  def test_03_task_scoop_circular(self):
    task_scoop_circular_result = test_arm_action(self,
      'TaskScoopCircular', owl_msgs.msg.TaskScoopCircularAction,
      owl_msgs.msg.TaskScoopCircularGoal(
        frame = 0,
        relative = False,
        point = Point(1.65, 0.0, constants.DEFAULT_GROUND_HEIGHT),
        depth = 0.01,
        parallel = True
      ),
      TEST_NAME, "test_03_task_scoop_circular"
    )

  def test_04_task_scoop_circular(self):
    task_scoop_circular_result = test_arm_action(self,
      'TaskScoopCircular', owl_msgs.msg.TaskScoopCircularAction,
      owl_msgs.msg.TaskScoopCircularGoal(
        frame = 0,
        relative = True,
        point = Point(-0.65, -0.02, -0.44),
        depth = 0.01,
        parallel = True
      ),
      TEST_NAME, "test_04_task_scoop_circular"
    )

  def test_05_task_scoop_circular(self):
    task_scoop_circular_result = test_arm_action(self,
      'TaskScoopCircular', owl_msgs.msg.TaskScoopCircularAction,
      owl_msgs.msg.TaskScoopCircularGoal(
        frame = 1,
        relative = True,
        point = Point(-0.8, -0.08, -0.4),
        depth = 0.03,
        parallel = True
      ),
      TEST_NAME, "test_05_task_scoop_circular"
    )

  def test_06_task_grind(self):
    task_grind_result = test_arm_action(self,
      'TaskGrind', owl_msgs.msg.TaskGrindAction,
      owl_msgs.msg.TaskGrindGoal(
        x_start = 1.65,
        y_start = 0.0,
        depth = 0.05,
        length = 0.6,
        parallel = False,
        ground_position = constants.DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, "test_06_task_grind"
    )

  def test_07_task_scoop_circular(self):
    task_scoop_circular_result = test_arm_action(self,
      'TaskScoopCircular', owl_msgs.msg.TaskScoopCircularAction,
      owl_msgs.msg.TaskScoopCircularGoal(
        frame = 0,
        relative = False,
        point = Point(1.65, 0.0, constants.DEFAULT_GROUND_HEIGHT),
        depth = 0.02,
        parallel = False
      ),
      TEST_NAME, "test_07_task_scoop_circular"
    )

  def test_08_task_grind(self):
    task_grind_result = test_arm_action(self,
      'TaskGrind', owl_msgs.msg.TaskGrindAction,
      owl_msgs.msg.TaskGrindGoal(
        x_start = 2.0,
        y_start = 0.6,
        depth = 0.1,
        length = 0.4,
        parallel = False,
        ground_position = constants.DEFAULT_GROUND_HEIGHT
      ),
      TEST_NAME, "test_08_task_grind"
    )

  def test_09_task_scoop_circular(self):
    task_scoop_circular_result = test_arm_action(self,
      'TaskScoopCircular', owl_msgs.msg.TaskScoopCircularAction,
      owl_msgs.msg.TaskScoopCircularGoal(
        frame = 0,
        relative = False,
        point = Point(2.0, 0.6, constants.DEFAULT_GROUND_HEIGHT),
        depth = 0.02,
        parallel = False
      ),
      TEST_NAME, "test_09_task_scoop_circular"
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestTaskScoopCircular)
