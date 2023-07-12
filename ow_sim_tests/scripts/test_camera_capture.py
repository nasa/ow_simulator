#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest
import owl_msgs.msg

from action_testing import *

PKG = 'ow_sim_tests'
TEST_NAME = 'camera_capture'
roslib.load_manifest(PKG)

class TestCameraCapture(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_camera_capture")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(10.0)

  def test_01_camera_capture(self):
    camera_capture_result = test_action(self,
      'CameraCapture', owl_msgs.msg.CameraCaptureAction,
      owl_msgs.msg.CameraCaptureGoal(),
      TEST_NAME, 'test_01_camera_capture',
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  # Move the antenna so the camera can see the robot arm.
  def test_02_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 1,
        point = Point(0, 0, 0)
      ),
      TEST_NAME, "test_02_pan_tilt_move_cartesian",
    )
  
  def test_03_camera_capture(self):
    camera_capture_result = test_action(self,
      'CameraCapture', owl_msgs.msg.CameraCaptureAction,
      owl_msgs.msg.CameraCaptureGoal(),
      TEST_NAME, 'test_03_camera_capture'
    )

  # Move the arm to unstow, then move the antenna, capture the image of the obot arm.
  def test_04_arm_unstow(self):
    arm_unstow_result = test_arm_action(self,
      'ArmUnstow', owl_msgs.msg.ArmUnstowAction,
      owl_msgs.msg.ArmUnstowGoal(),
      TEST_NAME, "test_04_arm_unstow",
    )

  def test_05_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 1,
        point = Point(0, 0, 0)
      ),
      TEST_NAME, "test_05_pan_tilt_move_cartesian",
    ) 

  def test_06_camera_capture(self):
    camera_capture_result = test_action(self,
      'CameraCapture', owl_msgs.msg.CameraCaptureAction,
      owl_msgs.msg.CameraCaptureGoal(),
      TEST_NAME, 'test_06_camera_capture'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestCameraCapture)
