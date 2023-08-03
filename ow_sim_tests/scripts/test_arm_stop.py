#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest
import owl_msgs.msg
import actionlib

from action_testing import *

PKG = 'ow_sim_tests'
TEST_NAME = 'arm_stop'
roslib.load_manifest(PKG)

# NOTE: Running this test at 3x real-time factor results in a slightly different
# log outcome, but otherwise the test still succeeds. This should be fixed by
# OW-1193

class TestArmStop(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_stop_test")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  # GoalStatus as ABORTED because arm_stop is not stop any action
  def test_01_arm_stop(self):
    arm_stop_result = test_arm_action(self,
      'ArmStop', owl_msgs.msg.ArmStopAction,
      owl_msgs.msg.ArmStopGoal(),
      TEST_NAME, "test_01_arm_stop",
      server_timeout = 50.0, # (seconds) first action call needs longer timeout,
      expected_end_state = actionlib.GoalStatus.ABORTED
    )

  def test_02_arm_unstow_and_stop(self):

    client_unstow = actionlib.SimpleActionClient('ArmUnstow', owl_msgs.msg.ArmUnstowAction)
    client_stop = actionlib.SimpleActionClient('ArmStop', owl_msgs.msg.ArmStopAction)

    connected_unstow = client_unstow.wait_for_server(timeout = rospy.Duration(10))
    connected_stop = client_stop.wait_for_server(timeout = rospy.Duration(10))
    
    client_unstow.send_goal(owl_msgs.msg.ArmUnstowGoal())
    client_unstow.wait_for_result(timeout=rospy.Duration.from_sec(5))

    client_stop.send_goal(owl_msgs.msg.ArmStopGoal())
    client_stop.wait_for_result()

    self.assertEqual(client_stop.get_goal_status_text(), "Arm trajectory stopped")

  def test_03_task_deliver_sample_and_stop(self):

    client_deliver = actionlib.SimpleActionClient('TaskDeliverSample', owl_msgs.msg.TaskDeliverSampleAction)
    client_stop = actionlib.SimpleActionClient('ArmStop', owl_msgs.msg.ArmStopAction)

    connected_unstow = client_deliver.wait_for_server(timeout = rospy.Duration(10))
    connected_stop = client_stop.wait_for_server(timeout = rospy.Duration(10))
    
    client_deliver.send_goal(owl_msgs.msg.TaskDeliverSampleGoal())
    client_deliver.wait_for_result(timeout=rospy.Duration.from_sec(30))

    client_stop.send_goal(owl_msgs.msg.ArmStopGoal())
    client_stop.wait_for_result()

    self.assertEqual(client_stop.get_goal_status_text(), "Arm trajectory stopped")

  def test_04_arm_stow_and_stop(self):

    client_stow = actionlib.SimpleActionClient('ArmStow', owl_msgs.msg.ArmUnstowAction)
    client_stop = actionlib.SimpleActionClient('ArmStop', owl_msgs.msg.ArmStopAction)

    connected_unstow = client_stow.wait_for_server(timeout = rospy.Duration(10))
    connected_stop = client_stop.wait_for_server(timeout = rospy.Duration(10))
    
    client_stow.send_goal(owl_msgs.msg.ArmStowGoal())
    client_stow.wait_for_result(timeout=rospy.Duration.from_sec(5))

    client_stop.send_goal(owl_msgs.msg.ArmStopGoal())
    client_stop.wait_for_result()

    self.assertEqual(client_stop.get_goal_status_text(), "Arm trajectory stopped")

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmStop)
