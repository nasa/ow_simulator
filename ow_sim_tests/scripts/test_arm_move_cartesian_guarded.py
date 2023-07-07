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
TEST_NAME = 'arm_move_cartesian_guarded'
roslib.load_manifest(PKG)

class TestArmMoveCartesianGuarded(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_arm_move_cartesian_guarded")

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

  def test_02_arm_move_cartesian_guarded(self):
    arm_move_cartesian_guarded_result = test_arm_action(self,
      'ArmMoveCartesianGuarded', owl_msgs.msg.ArmMoveCartesianGuardedAction,
      owl_msgs.msg.ArmMoveCartesianGuardedGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.86, -0.63, constants.DEFAULT_GROUND_HEIGHT + 0.2),
          orientation = Quaternion(1, 0.005, 0.053, 0.029)
        ),
        force_threshold = 200,
        torque_threshold = 100
      ),
      TEST_NAME, 'test_02_arm_move_cartesian_guarded'
    )

  def test_03_arm_move_cartesian_guarded(self):
    arm_move_cartesian_guarded_result = test_arm_action(self,
      'ArmMoveCartesianGuarded', owl_msgs.msg.ArmMoveCartesianGuardedAction,
      owl_msgs.msg.ArmMoveCartesianGuardedGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.86, -0.63, constants.DEFAULT_GROUND_HEIGHT - 0.1),
          orientation = Quaternion(1, 0.005, 0.053, 0.029)
        ),
        force_threshold = 200,
        torque_threshold = 100
      ),
      TEST_NAME, 'test_03_arm_move_cartesian_guarded'
    )

  def test_04_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.86, -0.63, constants.DEFAULT_GROUND_HEIGHT + 0.2),
          orientation = Quaternion(1, 0.005, 0.053, 0.029)
        ),
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
          position = Point(2.263, 0.147, -0.140),
          orientation = Quaternion(-0.615, 0.626, 0.339, 0.339)
        ),
      ),
      TEST_NAME, 'test_05_arm_move_cartesian'
    )

  def test_06_arm_move_cartesian_guarded(self):
    arm_move_cartesian_guarded_result = test_arm_action(self,
      'ArmMoveCartesianGuarded', owl_msgs.msg.ArmMoveCartesianGuardedAction,
      owl_msgs.msg.ArmMoveCartesianGuardedGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(2.263, -0.072, -0.157),
          orientation = Quaternion(0.707, -0.707, 0, 0)
        ),
        force_threshold = 200,
        torque_threshold = 5
      ),
      TEST_NAME, 'test_06_arm_move_cartesian_guarded'
    )
  
  def test_07_arm_move_cartesian_guarded(self):
    arm_move_cartesian_guarded_result = test_arm_action(self,
      'ArmMoveCartesianGuarded', owl_msgs.msg.ArmMoveCartesianGuardedAction,
      owl_msgs.msg.ArmMoveCartesianGuardedGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(2.263, -0.072, -0.157),
          orientation = Quaternion(0.707, -0.707, 0, 0)
        ),
        force_threshold = 200,
        torque_threshold = 50
      ),
      TEST_NAME, 'test_07_arm_move_cartesian_guarded'
    )

  def test_08_arm_move_cartesian_guarded(self):
    arm_move_cartesian_guarded_result = test_arm_action(self,
      'ArmMoveCartesianGuarded', owl_msgs.msg.ArmMoveCartesianGuardedAction,
      owl_msgs.msg.ArmMoveCartesianGuardedGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.797, 1.277, -0.160),
          orientation = Quaternion(0.959, -0.284, 0.003, 0.006)
        ),
        force_threshold = 200,
        torque_threshold = 10
      ),
      TEST_NAME, 'test_08_arm_move_cartesian_guarded'
    )

  def test_09_arm_move_cartesian_guarded(self):
    arm_move_cartesian_guarded_result = test_arm_action(self,
      'ArmMoveCartesianGuarded', owl_msgs.msg.ArmMoveCartesianGuardedAction,
      owl_msgs.msg.ArmMoveCartesianGuardedGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.797, 1.277, -0.160),
          orientation = Quaternion(0.959, -0.284, 0.003, 0.006)
        ),
        force_threshold = 200,
        torque_threshold = 50
      ),
      TEST_NAME, 'test_09_arm_move_cartesian_guarded'
    )

  def test_10_arm_move_cartesian_guarded(self):
    arm_move_cartesian_guarded_result = test_arm_action(self,
      'ArmMoveCartesianGuarded', owl_msgs.msg.ArmMoveCartesianGuardedAction,
      owl_msgs.msg.ArmMoveCartesianGuardedGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(2.263, -0.072, -0.157),
          orientation = Quaternion(0.707, -0.707, 0, 0)
        ),
        force_threshold = 200,
        torque_threshold = 50
      ),
      TEST_NAME, 'test_10_arm_move_cartesian_guarded'
    )

  def test_11_arm_move_cartesian_guarded(self):
    arm_move_cartesian_guarded_result = test_arm_action(self,
      'ArmMoveCartesianGuarded', owl_msgs.msg.ArmMoveCartesianGuardedAction,
      owl_msgs.msg.ArmMoveCartesianGuardedGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(2.265, 0.167, -0.140),
          orientation = Quaternion(0.625, -0.616, -0.338, -0.339)
        ),
        force_threshold = 200,
        torque_threshold = 30
      ),
      TEST_NAME, 'test_11_arm_move_cartesian_guarded'
    )

  def test_99_arm_stow(self):
    arm_stow_result = test_arm_action(self,
      'ArmStow', owl_msgs.msg.ArmStowAction,
      owl_msgs.msg.ArmStowGoal(),
      TEST_NAME, 'test_99_arm_stow'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmMoveCartesianGuarded)
