#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from math import sqrt
from copy import copy
import unittest
import rospy
import roslib
import actionlib
import numpy as np
import sys

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point

import owl_msgs.msg

from action_testing import *

PKG = 'ow_sim_tests'
TEST_NAME = 'sample_collection'
roslib.load_manifest(PKG)

GROUND_POSITION = -0.155
DISCARD_POSITION = Point(1.5, 0.8, GROUND_POSITION) # default for discard action
REGOLITH_CLEANUP_DELAY = 5.0 # seconds

SCOOP_LINK_NAME = 'lander::l_scoop'
SAMPLE_DOCK_LINK_NAME = 'lander::lander_sample_dock_link'

REGOLITH_PREFIX = 'regolith_'

"""
Computes the distance in only the X-Y plane between two geometry_msgs.msg Points
"""
def distance_flat_xy(p1, p2):
  v = Point(p2.x - p1.x,
            p2.y - p1.y,
            0)
  return sqrt(v.x*v.x + v.y*v.y)

class SampleCollection(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_check_action_test")

    # subscribe to get all gazebo link positions
    cls._gz_link_names = []
    cls._gz_link_poses = []
    cls._link_states_sub = rospy.Subscriber("/gazebo/link_states",
                                             LinkStates,
                                             cls._on_link_states)

    # changes behavior of test_action and test_arm_action
    set_ignore_action_checks('--ignore_action_checks' in sys.argv)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  """
  Callback function for the link_states topic
  """
  @classmethod
  def _on_link_states(cls, message):
    cls._gz_link_names = message.name
    cls._gz_link_poses = message.pose

  """
  Returns true if an action is done
  @param action_client: ROS action client object
  """
  def _is_action_done(self, action_client):
    return action_client.simple_state == actionlib.SimpleGoalState.DONE

  """
  Returns true if model/link name matches the regolith prefix
  @param name: Name of a model/link
  """
  def _is_regolith(self, name):
    return name.startswith(REGOLITH_PREFIX)

  """
  Returns the position of a givne link
  @param link_name: Name of a valid gazebo link
  """
  def _get_link_position(self, link_name):
    try:
      i = self._gz_link_names.index(link_name)
    except ValueError as e:
      rospy.logwarn(e)
      return None
    else:
      # make a shallow copy
      # otherwise functions that call this can modify positions by reference
      return copy(self._gz_link_poses[i].position)

  """
  Asserts that there are no regolith models present in the Gazebo world
  @param msg: Assert message
  """
  def _assert_regolith_not_present(self, msg="Regolith models spawned at wrong time!"):
    # Verify no regolith models are present
    self.assertFalse(
      any([self._is_regolith(name) for name in self._gz_link_names]),
      msg
    )

  """
  Asserts that there are regolith models present in the Gazebo world
  @param msg: Assert message
  """
  def _assert_regolith_present(self, msg="No regolith models spawned!"):
    self.assertTrue(
      any([self._is_regolith(name) for name in self._gz_link_names]),
      msg
    )

  """
  Asserts two Points are near each
  @param  p1: First point
  @param  p2: Second point
  @param  delta: Distance under which points are considered "near"
  @param  msg: Assert message
  """
  def _assert_point_is_near(self, p1, p2, delta, msg=None):
    self.assertLessEqual(distance(p1, p2), delta, msg)

  """
  Asserts two Points are far enough from each
  @param  p1: First point
  @param  p2: Second point
  @param  delta: Distance above which points are considered "far"
  @param  msg: Assert message
  """
  def _assert_point_is_far(self, p1, p2, delta, msg=None):
    self.assertGreater(distance(p1, p2), delta, msg)

  """
  Assert that, if regolith exists, it is in the scoop
  """
  def _assert_scoop_regolith_containment(self, assert_regolith_in_scoop=True):
    SCOOP_CONTAINMENT_RADIUS = 0.16 # meters
    scoop_position = self._get_link_position(SCOOP_LINK_NAME)
    if scoop_position is None:
      return
    # setup assertion and assertion message conditionally
    assert_func = self._assert_point_is_far
    assert_msg = "Regolith remains in scoop!\n"
    if assert_regolith_in_scoop:
      assert_func = self._assert_point_is_near
      assert_msg = "Regolith fell out of scoop!\n"
    assert_msg += ( "regolith name     : %s\n"
                    "regolith position : (%.2f, %.2f, %.2f)\n"
                    "scoop position    : (%.2f, %.2f, %.2f)"  )
    # Verify regolith models remain in the scoop after spawning
    for name, pose in zip(self._gz_link_names, self._gz_link_poses):
      if self._is_regolith(name):
        assert_func(
          pose.position, scoop_position, SCOOP_CONTAINMENT_RADIUS,
          assert_msg % (
                name,
                pose.position.x, pose.position.y, pose.position.z,
                scoop_position.x, scoop_position.y, scoop_position.z
              )
        )

  """
  check regolith in sample dock
  """
  def check_regolith_in_sample_dock(self, p):
    # define point as vector
    point = (p.x, p.y, p.z)
    # define 4 vertices P1 P2 P3 P4
    P1 = ( 0.6 ,   -0.425,  -6.53 )
    P2 = ( 0.505 , -0.425,  -6.53 )
    P3 = ( 0.6,    -0.13,   -6.61 )
    P4 = ( 0.6,    -0.425,  -6.47 )
    # define three directions U V W
    U = np.subtract(P2, P1)
    V = np.subtract(P3, P1)
    W = np.subtract(P4, P1)
    # Check following dot product constraints
    if (  U @ point > P1 @ U and U @ point < P2 @ U 
      and V @ point > P1 @ V and V @ point < P3 @ V
      and W @ point > P1 @ W and W @ point < P4 @ W):
      return True
    else:
      return False

  """
  Asserts all regolith contained in the sample dock
  """
  def _assert_dock_regolith_containment(self):
      #Verify all regolith remain in the sample dock
    for name, pose in zip(self._gz_link_names, self._gz_link_poses):
      assert_fail_msg = "Regolith fell out of dock!\n"
      if self._is_regolith(name):
        self.assertTrue(self.check_regolith_in_sample_dock(pose.position), assert_fail_msg)

  """
  Asserts regolith remains in scoop until it arrives at the sample dock
  """
  def _assert_regolith_transports_and_delivers_to_dock(self):
    # while scoop is transitioning to sample dock ensure reoglith remains inside
    scoop_position = self._get_link_position(SCOOP_LINK_NAME)
    dock_position = self._get_link_position(SAMPLE_DOCK_LINK_NAME)
    if scoop_position is None or dock_position is None:
      return
    # check if scoop is still transitioning to sample dock
    if distance_flat_xy(scoop_position, dock_position) > 0.2:
      self._assert_scoop_regolith_containment(True)

  """
  Asserts regolith remains in scoop until it arrives at the discard position
  """
  def _assert_regolith_transports_and_discards(self):
    scoop_position = self._get_link_position(SCOOP_LINK_NAME)
    if scoop_position is None:
      return
    # check if scoop is still transitioning to discard position
    if distance_flat_xy(scoop_position, DISCARD_POSITION) > 0.4:
      self._assert_scoop_regolith_containment(True)

  """
  Test the unstow action. Calling this first is standard operating procedure.
  """
  def test_01_unstow(self):
    unstow_result = test_arm_action(self,
      'ArmUnstow', owl_msgs.msg.ArmUnstowAction,
      owl_msgs.msg.ArmUnstowGoal(),
      TEST_NAME, "test_01_unstow",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  """
  Tests an extra deep grind action and asserts no regolith is spawned during the
  operation.
  """
  def test_02_grind(self):
    grind_result = test_arm_action(self,
      'TaskGrind', owl_msgs.msg.TaskGrindAction,
      owl_msgs.msg.TaskGrindGoal(
        x_start         = 1.65,
        y_start         = 0.0,
        depth           = 0.15, # grind deep so terrain is modified
        length          = 0.7,
        parallel        = True,
        ground_position = GROUND_POSITION
      ),
      TEST_NAME, "test_02_grind",
      condition_check = self._assert_regolith_not_present
    )

  """
  Tests a default dig linear action and asserts regolith does spawn during the
  operation and remains in the scoop until the end of it.
  """
  def test_03_dig_linear(self):
    dig_linear_result = test_arm_action(self,
      'TaskScoopLinear', owl_msgs.msg.TaskScoopLinearAction,
      owl_msgs.msg.TaskScoopLinearGoal(
        frame     = 0,
        relative  = False,
        point     = Point(1.46, 0, 0),
        depth     = 0.01,
        length    = 0.1
      ),
      TEST_NAME, "test_03_dig_linear",
      condition_check = self._assert_scoop_regolith_containment
    )

    # verify regolith models spawned
    self._assert_regolith_present()

  """
  Discards material over the default discard position and asserts it leaves
  scoop and is cleaned up shortly after hitting the terrain.
  """
  def test_04_discard(self):
    discard_result = test_arm_action(self,
      'TaskDiscardSample', owl_msgs.msg.TaskDiscardSampleAction,
      owl_msgs.msg.TaskDiscardSampleAction(
        frame = 0,
        relative = False,
        point = DISCARD_POSITION,
        height = 0.7
      ),
      TEST_NAME, "test_04_discard",
      condition_check = self._assert_regolith_transports_and_discards
    )

    # assert regolith fell out of scoop following the discard action
    self._assert_scoop_regolith_containment(False)

    # assert regolith was cleaned up after contacting with the terrain
    rospy.sleep(REGOLITH_CLEANUP_DELAY)
    self._assert_regolith_not_present()

  """
  Grind a new trench in a different location on the terrain.
  """
  def test_05_grind(self):
    grind_result = test_arm_action(self,
      'TaskGrind', owl_msgs.msg.TaskGrindAction,
      owl_msgs.msg.GrindGoal(
        x_start         = 1.65,
        y_start         = 0.5,
        depth           = 0.05,
        length          = 0.6,
        parallel        = True,
        ground_position = GROUND_POSITION
      ),
      TEST_NAME, "test_05_grind",
      condition_check = self._assert_regolith_not_present
    )

  """
  Dig for more material in the new trench, so we can then test delivery into
  the sample dock.
  """
  def test_06_dig_circular(self):
    dig_circular_result = test_arm_action(self,
      'TaskScoopCircular', owl_msgs.msg.TaskScoopCircularAction,
      owl_msgs.msg.TaskScoopCircularGoal(
        frame    = 0,
        relative = False,
        point    = Point(1.65, 0.5, 0.01),
        depth    = 0.01,
        parallel = True
      ),
      TEST_NAME, "test_06_dig_circular",
      condition_check = self._assert_scoop_regolith_containment
    )

    self._assert_regolith_present()

  """
  Tests a deliver action and asserts that regolith remains in the scoop until
  the final portion when it is dumped into the sample dock. Upon completion of
  the operation it also asserts that regolith models are deleted.
  """
  @unittest.expectedFailure
  def test_07_deliver(self):
    deliver_result = test_arm_action(self,
      'TaskDeliverSample', owl_msgs.msg.TaskDeliverSampleAction,
      owl_msgs.msg.TaskDeliverSampleGoal(),
      TEST_NAME, "test_07_deliver",
      condition_check = self._assert_regolith_transports_and_delivers_to_dock
    )

    # verify regolith has fallen out of the scoop
    self._assert_scoop_regolith_containment(False)
    self._assert_dock_regolith_containment()

  """
  Test the unstow action.
  Calling this after an optertion is standard operating procedure.
  """
  def test_08_unstow(self):
    unstow_result = test_arm_action(self,
      'ArmUnstow', owl_msgs.msg.ArmUnstowAction,
      owl_msgs.msg.ArmUnstowGoal(),
      TEST_NAME, "test_08_unstow"
    )

  """
  Test the stow action.
  Calling this after an optertion is standard operating procedure.
  """
  def test_09_stow(self):
    stow_result = test_arm_action(self,
      'ArmStow', owl_msgs.msg.ArmStowAction,
      owl_msgs.msg.ArmStowGoal(),
      TEST_NAME, "test_09_stow"
    )

  """
  Test the ingest sample action. Upon completion, ensure no sample remains in
  the simulation.
  """
  @unittest.expectedFailure
  def test_10_ingest_sample(self):
    ingest_result = test_action(self,
      'DockIngestSampleAction', owl_msgs.msg.DockIngestSampleAction,
      owl_msgs.msg.DockIngestSampleGoal(),
      TEST_NAME, "test_10_ingest_sample"
    )

    rospy.sleep(REGOLITH_CLEANUP_DELAY)
    self._assert_regolith_not_present(
      "Regolith remains after sample dock ingest action! It was either\n"
      "not properly delivered into the sample dock or the ingest sample\n"
      "action failed."
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, SampleCollection)
