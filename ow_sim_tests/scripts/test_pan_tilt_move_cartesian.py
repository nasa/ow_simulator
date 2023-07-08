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

PKG = 'ow_sim_tests'
TEST_NAME = 'pan_tilt_move_cartesian'
roslib.load_manifest(PKG)

class TestPanTiltMoveCartesian(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_unstow_test")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_arm_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 0,
        point = Point(1, 0, 0)
      ),
      TEST_NAME, "test_01_pan_tilt_move_cartesian",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    ) 

  def test_02_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_arm_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 0,
        point = Point(0, 0, 0)
      ),
      TEST_NAME, "test_02_pan_tilt_move_cartesian",
    ) 

  def test_03_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_arm_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 0,
        point = Point(7, 7, 7)
      ),
      TEST_NAME, "test_03_pan_tilt_move_cartesian",
    ) 

  def test_04_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_arm_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 1,
        point = Point(0, 0, 0)
      ),
      TEST_NAME, "test_04_pan_tilt_move_cartesian",
    ) 

  def test_05_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_arm_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 1,
        point = Point(1, 0, 0)
      ),
      TEST_NAME, "test_05_pan_tilt_move_cartesian",
    ) 

  def test_06_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_arm_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 1,
        point = Point(7, 7, 7)
      ),
      TEST_NAME, "test_06_pan_tilt_move_cartesian",
    ) 

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestPanTiltMoveCartesian)
