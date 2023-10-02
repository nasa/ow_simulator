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
TEST_NAME = 'task_discard_sample'
roslib.load_manifest(PKG)

class TestTaskDiscardSample(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_task_discard_sample")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 0,
        relative = False,
        point = Point(2.1, 0.2, constants.DEFAULT_GROUND_HEIGHT),
        height = 0.7
      ),
      TEST_NAME, "test_01_task_discard_sample",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 0,
        relative = False,
        point = Point(2.1, -0.1, constants.DEFAULT_GROUND_HEIGHT),
        height = 0.5
      ),
      TEST_NAME, "test_02_task_discard_sample"
    )

  def test_03_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 0,
        relative = False,
        point = Point(2.0, -0.3, constants.DEFAULT_GROUND_HEIGHT),
        height = 0.7
      ),
      TEST_NAME, "test_03_task_discard_sample"
    )

  def test_04_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 0,
        relative = True,
        point = Point(-0.1, -0.2, 0),
        height = 0.0
      ),
      TEST_NAME, "test_04_task_discard_sample"
    )

  def test_05_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 0,
        relative = True,
        point = Point(-0.3, -0.15, 0.1),
        height = 0.1
      ),
      TEST_NAME, "test_05_task_discard_sample"
    )

  def test_06_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 1,
        relative = False,
        point = Point(0, -0.4, 0.1),
        height = 0.0
      ),
      TEST_NAME, "test_06_task_discard_sample"
    )

  def test_07_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 1,
        relative = False,
        point = Point(0.1, -0.1, 0.1),
        height = 0.2
      ),
      TEST_NAME, "test_07_task_discard_sample"
    )

  def test_08_task_discard_sample(self):
    task_discard_sample_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleGoal(
        frame = 0,
        relative = False,
        point = Point(1.5, 0.8, constants.DEFAULT_GROUND_HEIGHT),
        height = 0.7
      ),
      TEST_NAME, "test_08_task_discard_sample"
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestTaskDiscardSample)
