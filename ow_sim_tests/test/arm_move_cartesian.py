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
TEST_NAME = 'arm_move_cartesian_action'
roslib.load_manifest(PKG)

class ArmMoveCartesianAction(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_unstow_test")

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

  def test_02_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(0.467, 0.160, 0.955),
          orientation = Quaternion(0.113, 0.904, 0.188, -0.366)
        )
      ),
      TEST_NAME, 'test_02_arm_move_cartesian'
    )

  def test_03_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(0.1281, -0.074, 0.839),
          orientation = Quaternion(0.691, -0.007, 0.156, 0.706)
        )
      ),
      TEST_NAME, 'test_03_arm_move_cartesian'
    )

  def test_04_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.166, 0.356, 0.588),
          orientation = Quaternion(-0.254, 0.727, -0.456, 0.446)
        )
      ),
      TEST_NAME, 'test_04_arm_move_cartesian'
    )

  def test_05_arm_move_cartesian(self):
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
      TEST_NAME, 'test_05_arm_move_cartesian'
    )

  def test_06_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(2.115, 0.345, 0.166),
          orientation = Quaternion(0.191, 0.455, -0.544, 0.678)
        )
      ),
      TEST_NAME, 'test_06_arm_move_cartesian'
    )

  def test_07_arm_stow(self):
    arm_stow_result = test_arm_action(self,
      'ArmStow', owl_msgs.msg.ArmStowAction,
      owl_msgs.msg.ArmStowGoal(),
      TEST_NAME, 'test_07_arm_stow'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, ArmMoveCartesianAction)
