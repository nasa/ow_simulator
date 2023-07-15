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
    # NOTE: This additional sleep is necessary due to an issue with a topic subscriber (see OW-1182)
    rospy.sleep(10)

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

  def test_07_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = True,
        joint = 5,
        angle = 0.261799
      ),
      TEST_NAME, 'test_07_arm_move_joint'
    )

  def test_08_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = True,
        joint = 4,
        angle = -0.436332
      ),
      TEST_NAME, 'test_08_arm_move_joint'
    )

  def test_09_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = True,
        joint = 3,
        angle = -0.314159
      ),
      TEST_NAME, 'test_09_arm_move_joint'
    )

  def test_10_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = True,
        joint = 2,
        angle = 0.314159
      ),
      TEST_NAME, 'test_10_arm_move_joint'
    )

  def test_11_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = True,
        joint = 1,
        angle = 0.174533
      ),
      TEST_NAME, 'test_11_arm_move_joint'
    )

  def test_12_arm_move_joint(self):
    arm_move_joint_result = test_arm_action(self,
      'ArmMoveJoint', owl_msgs.msg.ArmMoveJointAction,
      owl_msgs.msg.ArmMoveJointGoal(
        relative = True,
        joint = 0,
        angle = 0.174533
      ),
      TEST_NAME, 'test_12_arm_move_joint'
    )


if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmMoveJoint)
