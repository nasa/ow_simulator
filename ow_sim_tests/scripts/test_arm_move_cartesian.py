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
TEST_NAME = 'arm_move_cartesian'
roslib.load_manifest(PKG)

class TestArmMoveCartesian(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_move_cartesian_unit_test")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.736, 0.255, 0.535),
          orientation = Quaternion(-0.499, 0.501, 0.501, 0.499)
        )
      ),
      TEST_NAME, 'test_01_arm_move_cartesian',
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.639, -0.996, 0.511),
          orientation = Quaternion(0.675, -0.465, 0.072, 0.568)
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
          position = Point(1.166, 0.356, 0.588),
          orientation = Quaternion(-0.254, 0.727, -0.456, 0.446)
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
          position = Point(1.833, 0.713, 0.782),
          orientation = Quaternion(-0.539, -0.332, 0.370, 0.681)
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
          position = Point(2.115, 0.345, 0.3),
          orientation = Quaternion(0.191, 0.455, -0.544, 0.678)
        )
      ),
      TEST_NAME, 'test_05_arm_move_cartesian'
    )

  def test_06_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 1,
        relative = False,
        pose= Pose(
          position = Point(0, 0.05, 0.05),
          orientation = Quaternion(0, 0, 0.383, 0.924)
        )
      ),
      TEST_NAME, 'test_06_arm_move_cartesian'
    )

  def test_07_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 1,
        relative = False,
        pose= Pose(
          position = Point(-0.2, 0, 0),
          orientation = Quaternion(-0.089, -0.215, 0.372, 0.899)
        )
      ),
      TEST_NAME, 'test_07_arm_move_cartesian'
    )

  def test_08_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = True,
        pose= Pose(
          position = Point(-0.02, 0.02, 0.1),
          orientation = Quaternion(0, 0, 0, 1)
        )
      ),
      TEST_NAME, 'test_08_arm_move_cartesian'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmMoveCartesian)
