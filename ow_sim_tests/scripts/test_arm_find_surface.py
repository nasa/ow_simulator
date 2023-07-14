#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import roslib
import unittest
import owl_msgs.msg

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

  def test_01_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.7, 0.5, constants.DEFAULT_GROUND_HEIGHT),
        normal = Vector3(0, 0, -1),
        distance = 0.2,
        overdrive = 0.05,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_01_arm_find_surface',
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

   # Uisng arm_move_cartesian after every arm_find_surface to get the arm off the ground
  def test_02_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = True,
        pose= Pose(
          position = Point(0, 0, 0.2),
          orientation = Quaternion(0, 0, 0, 1)
        )
      ),
      TEST_NAME, 'test_02_arm_move_cartesian'
    )

  def test_03_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.86, 0.1, constants.DEFAULT_GROUND_HEIGHT),
        normal = Vector3(0, 0, -1),
        distance = 0.5,
        overdrive = 0.2,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_03_arm_find_surface'
    )

  def test_04_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = True,
        pose= Pose(
          position = Point(0, 0, 0.2),
          orientation = Quaternion(0, 0, 0, 1)
        )
      ),
      TEST_NAME, 'test_04_arm_move_cartesian'
    )

  def test_05_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.5, -0.01, constants.DEFAULT_GROUND_HEIGHT),
        normal = Vector3(0, 0, -1),
        distance = 0.2,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_05_arm_find_surface'
    )

  def test_06_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = True,
        pose= Pose(
          position = Point(0, 0, 0.2),
          orientation = Quaternion(0, 0, 0, 1)
        )
      ),
      TEST_NAME, 'test_06_arm_move_cartesian'
    )

  def test_07_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.8, -0.3, constants.DEFAULT_GROUND_HEIGHT),
        normal = Vector3(0, 0, -1),
        distance = 0.2,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_07_arm_find_surface'
    )

  def test_08_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = True,
        pose= Pose(
          position = Point(0, 0, 0.2),
          orientation = Quaternion(0, 0, 0, 1)
        )
      ),
      TEST_NAME, 'test_08_arm_move_cartesian'
    )

  def test_09_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(2, 0.17, constants.DEFAULT_GROUND_HEIGHT),
        normal = Vector3(0, 0, -1),
        distance = 0.2,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_09_arm_find_surface'
    )

  def test_10_arm_move_cartesian(self):
    arm_move_cartesian_result = test_arm_action(self,
      'ArmMoveCartesian', owl_msgs.msg.ArmMoveCartesianAction,
      owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = True,
        pose= Pose(
          position = Point(0, 0, 0.2),
          orientation = Quaternion(0, 0, 0, 1)
        )
      ),
      TEST_NAME, 'test_10_arm_move_cartesian'
    )

  def test_11_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.86, -0.63, constants.DEFAULT_GROUND_HEIGHT + 0.5),
        # The +0.5 here simulates a situation the arm can not find a surface
        normal = Vector3(0, 0, -1),
        distance = 0.1,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 200
      ),
      TEST_NAME, 'test_11_arm_find_surface'
    )
  
  def test_12_arm_find_surface(self):
    arm_find_surface_result = test_arm_action(self,
      'ArmFindSurface', owl_msgs.msg.ArmFindSurfaceAction,
      owl_msgs.msg.ArmFindSurfaceGoal(
        frame = 0,
        relative = False,
        position = Point(1.86, -0.63, constants.DEFAULT_GROUND_HEIGHT),
        normal = Vector3(-0.1, -0.3, -0.8124),
        distance = 0.2,
        overdrive = 0.1,
        force_threshold = 200,
        torque_threshold = 50
      ),
      TEST_NAME, 'test_12_arm_find_surface'
    ) 

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmFindSurface)
