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
TEST_NAME = 'arm_move_joints_guarded'
roslib.load_manifest(PKG)

class TestArmMoveJointsGuarded(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("test_arm_move_joints_guarded")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_arm_move_joints_guarded(self):
    arm_move_joints_guarded_result = test_arm_action(self,
      'ArmMoveJointsGuarded', owl_msgs.msg.ArmMoveJointsGuardedAction,
      owl_msgs.msg.ArmMoveJointsGuardedGoal(
        relative = False,
        angles = [0.0349066, 0.890118, -1.78024, 0.907571, 0.994838, -0.0174533],
        force_threshold = 200,
        torque_threshold = 100
      ),
      TEST_NAME, 'test_01_arm_move_joints_guarded',
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_arm_move_joints_guarded(self):
    arm_move_joints_guarded_result = test_arm_action(self,
      'ArmMoveJointsGuarded', owl_msgs.msg.ArmMoveJointsGuardedAction,
      owl_msgs.msg.ArmMoveJointsGuardedGoal(
        relative = True,
        angles = [0, 0, 0, 0, -0.6, 0],
        force_threshold = 50,
        torque_threshold = 30
      ),
      TEST_NAME, 'test_02_arm_move_joints_guarded'
    )
    
  def test_03_arm_move_joints_guarded(self):
    arm_move_joints_guarded_result = test_arm_action(self,
      'ArmMoveJointsGuarded', owl_msgs.msg.ArmMoveJointsGuardedAction,
      owl_msgs.msg.ArmMoveJointsGuardedGoal(
        relative = True,
        angles = [-0.2, 0, 0, 0, 0, 0],
        force_threshold = 200,
        torque_threshold = 100
      ),
      TEST_NAME, 'test_03_arm_move_joints_guarded'
    )

  def test_04_arm_move_joints_guarded(self):
    arm_move_joints_guarded_result = test_arm_action(self,
      'ArmMoveJointsGuarded', owl_msgs.msg.ArmMoveJointsGuardedAction,
      owl_msgs.msg.ArmMoveJointsGuardedGoal(
        relative = True,
        angles = [-0.2, 0, 0, 0, 0, 0],
        force_threshold = 50,
        torque_threshold = 100
      ),
      TEST_NAME, 'test_04_arm_move_joints_guarded'
    )
    
if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmMoveJointsGuarded)
