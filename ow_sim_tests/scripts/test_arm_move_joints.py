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
TEST_NAME = 'arm_move_joints'
roslib.load_manifest(PKG)

class TestArmMoveJoints(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_arm_move_joints")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_arm_move_joints(self):
    arm_move_joints_result = test_arm_action(self,
      'ArmMoveJoints', owl_msgs.msg.ArmMoveJointsAction,
      owl_msgs.msg.ArmMoveJointsGoal(
        relative = False,
        angles = [0, 1.5708, -1.5708, 0, 1.5708, 0]
      ),
      TEST_NAME, 'test_01_arm_move_joints',
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_arm_move_joints(self):
    arm_move_joints_result = test_arm_action(self,
      'ArmMoveJoints', owl_msgs.msg.ArmMoveJointsAction,
      owl_msgs.msg.ArmMoveJointsGoal(
        relative = False,
        angles = [0, 0, 0, 0, 0, 0]
      ),
      TEST_NAME, 'test_02_arm_move_joints'
    )
    
  def test_03_arm_move_joints(self):
    arm_move_joints_result = test_arm_action(self,
      'ArmMoveJoints', owl_msgs.msg.ArmMoveJointsAction,
      owl_msgs.msg.ArmMoveJointsGoal(
        relative = False,
        angles = [0.837758, 1.48353, 7.87143, -0.314159, 3.47321, 2.28638]
      ),
      TEST_NAME, 'test_03_arm_move_joints'
    )

  def test_04_arm_move_joints(self):
    arm_move_joints_result = test_arm_action(self,
      'ArmMoveJoints', owl_msgs.msg.ArmMoveJointsAction,
      owl_msgs.msg.ArmMoveJointsGoal(
        relative = False,
        angles = [0.750492, 1.23918, 8.08087, 0.0872665, -3.15905, 2.37365]
      ),
      TEST_NAME, 'test_04_arm_move_joints'
    )

  def test_05_arm_move_joints(self):
    arm_move_joints_result = test_arm_action(self,
      'ArmMoveJoints', owl_msgs.msg.ArmMoveJointsAction,
      owl_msgs.msg.ArmMoveJointsGoal(
        relative = False,
        angles = [0.977384, 1.78024, 0.959931, 1.97222, 2.54818, 1.5708]
      ),
      TEST_NAME, 'test_05_arm_move_joints'
    )

  def test_99_arm_move_joints(self):
    arm_move_joints_result = test_arm_action(self,
      'ArmMoveJoints', owl_msgs.msg.ArmMoveJointsAction,
      owl_msgs.msg.ArmMoveJointsGoal(
        relative = False,
        angles = [-1.50098, 1.5708, -2.6529, 2.89725, 0, 0]
      ),
      TEST_NAME, 'test_99_arm_move_joints'
    )
if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmMoveJoints)
