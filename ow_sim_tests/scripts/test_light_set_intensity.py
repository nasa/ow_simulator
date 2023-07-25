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
TEST_NAME = 'light_set_intensity'
roslib.load_manifest(PKG)

class TestLightSetIntensity(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_light_set_intensity")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'left',
        intensity = 0.2
      ),
      TEST_NAME, 'test_01_light_set_intensity',
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'right',
        intensity = 0.2
      ),
      TEST_NAME, 'test_01_light_set_intensity'
    )

  # Move the antenna pointing to the base_link for light intensity visualization
  def test_03_pan_tilt_move_cartesian(self):
    pan_tilt_move_cartesian_result = test_action(self,
      'PanTiltMoveCartesian', owl_msgs.msg.PanTiltMoveCartesianAction,
      owl_msgs.msg.PanTiltMoveCartesianGoal(
        frame = 0,
        point = Point(0, 0, 0)
      ),
      TEST_NAME, "test_03_pan_tilt_move_cartesian",
    )
  
  def test_04_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'left',
        intensity = 0.4
      ),
      TEST_NAME, 'test_04_light_set_intensity'
    )
    rospy.sleep(3.0)

  def test_05_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'left',
        intensity = 0.6
      ),
      TEST_NAME, 'test_05_light_set_intensity'
    )
    rospy.sleep(3.0)
  
  def test_06_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'left',
        intensity = 0.8
      ),
      TEST_NAME, 'test_06_light_set_intensity'
    )
    rospy.sleep(3.0)

  def test_07_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'left',
        intensity = 1.0
      ),
      TEST_NAME, 'test_07_light_set_intensity'
    )
    rospy.sleep(3.0)

  def test_08_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'right',
        intensity = 0.4
      ),
      TEST_NAME, 'test_08_light_set_intensity'
    )
    rospy.sleep(3.0)

  def test_09_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'right',
        intensity = 0.6
      ),
      TEST_NAME, 'test_09_light_set_intensity'
    )
    rospy.sleep(3.0)
  
  def test_10_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'right',
        intensity = 0.8
      ),
      TEST_NAME, 'test_10_light_set_intensity'
    )
    rospy.sleep(3.0)

  def test_11_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'right',
        intensity = 1.0
      ),
      TEST_NAME, 'test_11_light_set_intensity'
    )
    rospy.sleep(3.0)

  def test_12_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'left',
        intensity = 0.0
      ),
      TEST_NAME, 'test_12_light_set_intensity'
    )
    rospy.sleep(3.0)

  def test_13_light_set_intensity(self):
    light_set_intensity_result = test_action(self,
      'LightSetIntensity', owl_msgs.msg.LightSetIntensityAction,
      owl_msgs.msg.LightSetIntensityGoal(
        name = 'right',
        intensity = 0.0
      ),
      TEST_NAME, 'test_13_light_set_intensity'
    )
    rospy.sleep(3.0)
    
if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestLightSetIntensity)
