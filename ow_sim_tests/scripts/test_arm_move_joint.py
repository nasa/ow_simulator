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
TEST_NAME = 'arm_move_joint'
roslib.load_manifest(PKG)

class TestArmMoveJoint(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_arm_move_joint")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = False,
        joint = 0,
        angle = 0
      ),
      TEST_NAME, 'test_01_arm_move_joint',
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    ) 
  
  def test_02_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = False,
        joint = 1,
        angle = 1.5708
      ),
      TEST_NAME, 'test_02_arm_move_joint'
    )

  def test_03_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = False,
        joint = 2,
        angle = -1.5708
      ),
      TEST_NAME, 'test_03_arm_move_joint'
    )

  def test_04_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = False,
        joint = 3,
        angle = 0
      ),
      TEST_NAME, 'test_04_arm_move_joint'
    )

  def test_05_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = False,
        joint = 4,
        angle = 1.5708
      ),
      TEST_NAME, 'test_05_arm_move_joint'
    )

  def test_06_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = False,
        joint = 5,
        angle = 0
      ),
      TEST_NAME, 'test_06_arm_move_joint'
    )

  def test_07_arm_stow(self):
    arm_stow_result = test_arm_action(self,
      'ArmStow', owl_msgs.msg.ArmStowAction,
      owl_msgs.msg.ArmStowGoal(),
      TEST_NAME, 'test_07_arm_stow'
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmMoveJoint)
