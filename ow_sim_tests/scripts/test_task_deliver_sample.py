#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest
import owl_msgs.msg

from action_testing import *
from geometry_msgs.msg import Pose, Quaternion

PKG = 'ow_sim_tests'
TEST_NAME = 'task_deliver_sample'
roslib.load_manifest(PKG)

class TestTaskDeliverSample(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_task_deliver_sample")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_task_deliver_sample(self):
    task_deliver_sample_result = test_arm_action(self,
      'TaskDeliverSample', owl_msgs.msg.TaskDeliverSampleAction,
      owl_msgs.msg.TaskDeliverSampleGoal(),
      TEST_NAME, "test_01_task_deliver_sample",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )
  
  # Move the arm to other location and test again
  def test_02_arm_unstow(self):
    arm_unstow_result = test_arm_action(self,
      'ArmUnstow', owl_msgs.msg.ArmUnstowAction,
      owl_msgs.msg.ArmUnstowGoal(),
      TEST_NAME, 'test_02_arm_unstow'
    )
  
  def test_03_task_deliver_sample(self):
    task_deliver_sample_result = test_arm_action(self,
      'TaskDeliverSample', owl_msgs.msg.TaskDeliverSampleAction,
      owl_msgs.msg.TaskDeliverSampleGoal(),
      TEST_NAME, "test_03_task_deliver_sample",
    )

  def test_04_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.833, 0.713, 0.782),
          orientation = Quaternion(-0.539, -0.332, 0.370, 0.681)
        )
      ),
      TEST_NAME, 'test_04_arm_move_cartesian'
    ) 

  def test_05_task_deliver_sample(self):
    task_deliver_sample_result = test_arm_action(self,
      'TaskDeliverSample', owl_msgs.msg.TaskDeliverSampleAction,
      owl_msgs.msg.TaskDeliverSampleGoal(),
      TEST_NAME, "test_05_task_deliver_sample",
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestTaskDeliverSample)
