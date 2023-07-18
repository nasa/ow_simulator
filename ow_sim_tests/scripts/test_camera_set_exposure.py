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
TEST_NAME = 'camera_set_exposure'
roslib.load_manifest(PKG)

class TestCameraSetExposure(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_camera_set_exposure")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)
  
  def test_01_camera_set_exposure(self):
    camera_set_exposure_result = test_action(self,
      'CameraSetExposure', owl_msgs.msg.CameraSetExposureAction,
      owl_msgs.msg.CameraSetExposureGoal(
        automatic = True,
        exposure = -1
      ),
      TEST_NAME, 'test_01_camera_set_exposure',
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
      TEST_NAME, "test_02_pan_tilt_move_cartesian"
    )
  
  def test_03_camera_capture(self):
    camera_capture_result = test_action(self,
      'CameraCapture', owl_msgs.msg.CameraCaptureAction,
      owl_msgs.msg.CameraCaptureGoal(),
      TEST_NAME, 'test_03_camera_capture'
    )

  def test_04_camera_set_exposure(self):
    camera_set_exposure_result = test_action(self,
      'CameraSetExposure', owl_msgs.msg.CameraSetExposureAction,
      owl_msgs.msg.CameraSetExposureGoal(
        automatic = False,
        exposure = 0.01
      ),
      TEST_NAME, 'test_04_camera_set_exposure'
    )

  def test_05_camera_capture(self):
    camera_capture_result = test_action(self,
      'CameraCapture', owl_msgs.msg.CameraCaptureAction,
      owl_msgs.msg.CameraCaptureGoal(),
      TEST_NAME, 'test_05_camera_capture'
    )

  def test_06_camera_set_exposure(self):
    camera_set_exposure_result = test_action(self,
      'CameraSetExposure', owl_msgs.msg.CameraSetExposureAction,
      owl_msgs.msg.CameraSetExposureGoal(
        automatic = False,
        exposure = 0.02
      ),
      TEST_NAME, 'test_06_camera_set_exposure'
    )

  def test_07_camera_capture(self):
    camera_capture_result = test_action(self,
      'CameraCapture', owl_msgs.msg.CameraCaptureAction,
      owl_msgs.msg.CameraCaptureGoal(),
      TEST_NAME, 'test_07_camera_capture'
    )

  def test_08_camera_set_exposure(self):
    camera_set_exposure_result = test_action(self,
      'CameraSetExposure', owl_msgs.msg.CameraSetExposureAction,
      owl_msgs.msg.CameraSetExposureGoal(
        automatic = False,
        exposure = 0.04
      ),
      TEST_NAME, 'test_08_camera_set_exposure'
    )

  def test_09_camera_capture(self):
    camera_capture_result = test_action(self,
      'CameraCapture', owl_msgs.msg.CameraCaptureAction,
      owl_msgs.msg.CameraCaptureGoal(),
      TEST_NAME, 'test_09_camera_capture'
    )

  def test_10_camera_set_exposure(self):
    camera_set_exposure_result = test_action(self,
      'CameraSetExposure', owl_msgs.msg.CameraSetExposureAction,
      owl_msgs.msg.CameraSetExposureGoal(
        automatic = False,
        exposure = 0.05
      ),
      TEST_NAME, 'test_10_camera_set_exposure'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestCameraSetExposure)
