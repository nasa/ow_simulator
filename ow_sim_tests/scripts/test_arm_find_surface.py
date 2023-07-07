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
from ow_lander import constants
from geometry_msgs.msg import Vector3, Pose, Quaternion

PKG = 'ow_sim_tests'
TEST_NAME = 'arm_find_surface'
roslib.load_manifest(PKG)

class TestArmFindSurface(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_arm_find_surface")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_arm_unstow(self):
    arm_unstow_result = test_arm_action(self,
      'ArmUnstow', owl_msgs.msg.ArmUnstowAction,
      owl_msgs.msg.ArmUnstowGoal(),
      TEST_NAME, "test_01_arm_unstow",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.7, 0.5, constants.DEFAULT_GROUND_HEIGHT + 0.2),
        normal = Vector3(0, 0, -1),
        distance = 0.2,
        overdrive = 0.05,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_02_arm_find_surface'
    )

  def test_03_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.7, 0.5, constants.DEFAULT_GROUND_HEIGHT + 0.2),
          orientation = Quaternion(1, 0.015, 0.040, -0.015)
        )
      ),
      TEST_NAME, 'test_03_arm_move_cartesian'
    )

  def test_04_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.86, -0.63, constants.DEFAULT_GROUND_HEIGHT + 0.5),
        normal = Vector3(0, 0, -1),
        distance = 0.5,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_04_arm_find_surface'
    )

  def test_05_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.86, -0.63, constants.DEFAULT_GROUND_HEIGHT + 0.2),
          orientation = Quaternion(1, 0.005, 0.053, 0.029)
        )
      ),
      TEST_NAME, 'test_05_arm_move_cartesian'
    )

  def test_06_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.5, -0.01, constants.DEFAULT_GROUND_HEIGHT + 0.2),
        normal = Vector3(0, 0, -1),
        distance = 0.2,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_06_arm_find_surface'
    )

  def test_07_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.5, -0.01, constants.DEFAULT_GROUND_HEIGHT + 0.2),
          orientation = Quaternion(1, 0.016, 0.035, 0)
        )
      ),
      TEST_NAME, 'test_07_arm_move_cartesian'
    )

  def test_08_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.8, -0.3, constants.DEFAULT_GROUND_HEIGHT + 0.2),
        normal = Vector3(0, 0, -1),
        distance = 0.2,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_08_arm_find_surface'
    )

  def test_09_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.8, -0.3, constants.DEFAULT_GROUND_HEIGHT + 0.2),
          orientation = Quaternion(1, 0.007, 0.059, 0)
        )
      ),
      TEST_NAME, 'test_09_arm_move_cartesian'
    )

  def test_10_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(2, 0.17, constants.DEFAULT_GROUND_HEIGHT + 0.2),
        normal = Vector3(0, 0, -1),
        distance = 0.2,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_10_arm_find_surface'
    )

  def test_11_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(2, 0.17, constants.DEFAULT_GROUND_HEIGHT + 0.2),
          orientation = Quaternion(1, -0.012, 0.058, 0)
        )
      ),
      TEST_NAME, 'test_11_arm_move_cartesian'
    )
    
  def test_12_arm_stow(self):
    arm_stow_result = test_arm_action(self,
      'ArmStow', owl_msgs.msg.ArmStowAction,
      owl_msgs.msg.ArmStowGoal(),
      TEST_NAME, 'test_12_arm_stow'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmFindSurface)
