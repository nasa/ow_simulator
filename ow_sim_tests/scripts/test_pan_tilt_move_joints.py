#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest
import owl_msgs.msg
import ow_lander.msg
from action_testing import *

PKG = 'ow_sim_tests'
TEST_NAME = 'pan_tilt_move_joints'
roslib.load_manifest(PKG)

class TestPanTiltMoveJoints(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("pan_tilt_move_joints_test")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = 0.785,
        tilt = 1
      ),
      TEST_NAME, "test_01_pan_tilt_move_joints",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    ) 

  def test_02_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = 0.785,
        tilt = 1
      ),
      TEST_NAME, "test_02_pan_tilt_move_joints"
    ) 

  def test_03_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = 0.785,
        tilt = 0.785
      ),
      TEST_NAME, "test_03_pan_tilt_move_joints"
    ) 

  def test_04_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = -0.785,
        tilt = -0.785
      ),
      TEST_NAME, "test_04_pan_tilt_move_joints"
    ) 

  def test_05_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = 0,
        tilt = -0.785
      ),
      TEST_NAME, "test_05_pan_tilt_move_joints"
    ) 

  def test_06_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = -0.785,
        tilt = 0
      ),
      TEST_NAME, "test_06_pan_tilt_move_joints"
    ) 

  def test_07_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = 3.1,
        tilt = 1.55
      ),
      TEST_NAME, "test_07_pan_tilt_move_joints"
    )

  def test_08_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = -3.1,
        tilt = -1.55
      ),
      TEST_NAME, "test_08_pan_tilt_move_joints"
    )

  def test_09_pan_tilt_move_joints(self):
    pan_tilt_move_joints_result = test_action(self,
      'PanTiltMoveJoints', owl_msgs.msg.PanTiltMoveJointsAction,
      owl_msgs.msg.PanTiltMoveJointsGoal(
        pan = 0,
        tilt = 0
      ),
      TEST_NAME, "test_09_pan_tilt_move_joints"
    )

  # Test 10 and 11 are employed to test pan.py and tilt.py
  def test_10_pan(self):
    pan_result = test_action(self,
      'Pan', ow_lander.msg.PanAction,
      ow_lander.msg.PanGoal(
        pan = 1
      ),
      TEST_NAME, "test_10_pan"
    )

  def test_11_tilt(self):
    tilt_result = test_action(self,
      'Tilt', ow_lander.msg.TiltAction,
      ow_lander.msg.TiltGoal(
        tilt = 1
      ),
      TEST_NAME, "test_11_tilt"
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestPanTiltMoveJoints)
