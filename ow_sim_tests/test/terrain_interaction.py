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

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point

import ow_lander.msg

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)

GROUND_POSITION = -0.155
DISCARD_POSITION = Point(1.5, 0.8, 0.65) # default for discard action
REGOLITH_CLEANUP_DELAY = 5.0 # seconds

SCOOP_LINK_NAME = 'lander::l_scoop'
SAMPLE_DOCK_LINK_NAME = 'lander::lander_sample_dock_link'

REGOLITH_PREFIX = 'regolith_'

"""
Computes the 3D distance between two geometry_msgs.msg Points
"""
def distance(p1, p2):
  v = Point(p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z)
  return sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

"""
Computes the distance in only the X-Y plane between two geometry_msgs.msg Points
"""
def distance_flat_xy(p1, p2):
  v = Point(p2.x - p1.x,
            p2.y - p1.y,
            0)
  return sqrt(v.x*v.x + v.y*v.y)

class TerrainInteraction(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node('terrain_interaction_test', anonymous=True)

    # subscribe to get all gazebo link positions
    self._gz_link_names = []
    self._gz_link_poses = []
    self._link_states_sub = rospy.Subscriber("/gazebo/link_states",
                                             LinkStates,
                                             self._on_link_states)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  """
  Callback function for the link_states topic
  """
  def _on_link_states(self, message):
    self._gz_link_names = message.name
    self._gz_link_poses = message.pose

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
  Asserts nothing. Pass this to _test_action a action_fail_condition is not
  necessary.
  """
  def _assert_nothing(self):
    pass

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
  @param p1: First point
  @param p2: Second point
  @param delta: Distance under which points are considered "near"
  @param msg: Assert message
  """
  def _assert_point_is_near(self, p1, p2, delta, msg):
    self.assertLessEqual(distance(p1, p2), delta, msg)

  """
  Asserts two Points are far enough from each
  @param p1: First point
  @param p2: Second point
  @param delta: Distance above which points are considered "far"
  @param msg: Assert message
  """
  def _assert_point_is_far(self, p1, p2, delta, msg):
    self.assertGreater(distance(p1, p2), delta, msg)

  """
  Assert that, if regolith exists, it is in the scoop
  """
  def _assert_scoop_regolith_containment(self, assert_regolith_in_scoop=True):
    SCOOP_CONTAINMENT_RADIUS = 0.12 # meters
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
  Calls an action asynchronously allowing checks to occur during its execution.
  @param action_name: Name of action to be called
  @param action: ROS action object
  @param goal: Goal object that kwargs populates
  @param max_duration: Max time allotted to action in seconds
  @param action_fail_condition: A function repeatedly called during action
         execution that asserts requirements for a successful action operation
  @param expected_final: Final arm position to be compared with action result
         (default: None)
  @param server_timeout: Time the action server is waited for (default: 10.0)
  @param condition_check_interval: The interval in seconds between calls to
         action_fail_condition (default: 0.2)
  @param expected_final_tolerance: allowed distance between result.final and
         expected_final (default: 0.02)
  @kwargs: Any properties of goal
  """
  def _test_action(self, action_name, action, goal, max_duration,
                   action_fail_condition,
                   expected_final=None,
                   server_timeout=10.0,
                   condition_check_interval=0.2,
                   expected_final_tolerance=0.02,
                   **kwargs):

    client = actionlib.SimpleActionClient(action_name, action)
    connected = client.wait_for_server(timeout=rospy.Duration(server_timeout))
    self.assertTrue(
      connected,
      "Timeout exceeded waiting for %s action server" % action_name
    )

    # populate goal parameters using kwargs
    parameter_list = ""
    for key, value in kwargs.items():
      if hasattr(goal, key):
        setattr(goal, key, value)
        parameter_list += "\n%s : %s" % (key, str(value))
      else:
        self.fail(
          "%s action does not contain parameter %s" % (action_name, key)
        )

    client.send_goal(goal)

    rospy.loginfo(
      "\n===%s action goal sent===%s" % (action_name, parameter_list)
    )

    # monitor for failed conditions during action execution
    start = rospy.get_time()
    elapsed = 0.0
    while not self._is_action_done(client) and elapsed < max_duration:
      action_fail_condition()
      rospy.sleep(condition_check_interval)
      elapsed = rospy.get_time() - start

    self.assertLess(
      elapsed, max_duration,
      "Timeout reached waiting for %s action to finish!" % action_name
    )

    result = client.get_result()

    rospy.loginfo(
      "\n===%s action completed in %0.3fs===" % (action_name, elapsed)
    )

    # verify action ended where we expected
    if expected_final is not None:
      self._assert_point_is_near(
        result.final, expected_final, expected_final_tolerance,
        "Arm did not complete the %s action in the position expected!"
          % action_name
      )

    return result

  """
  Test the unstow action. Calling this first is standard operating procedure.
  """
  def test_01_unstow(self):

    UNSTOW_MAX_DURATION = 30.0
    UNSTOW_EXPECTED_FINAL = Point(1.7419, 0.2396, -6.5904)

    unstow_result = self._test_action(
      'Unstow',
      ow_lander.msg.UnstowAction,
      ow_lander.msg.UnstowGoal(),
      UNSTOW_MAX_DURATION,
      self._assert_nothing,
      expected_final = UNSTOW_EXPECTED_FINAL,
      server_timeout = 25.0, # (seconds) first action call needs longer timeout
      expected_final_tolerance = 0.5 # unstow requires a really high tolerance
    )

  """
  Tests an extra deep grind action and asserts no regolith is spawned during the
  operation.
  """
  def test_02_grind(self):

    GRIND_MAX_DURATION = 80.0 # seconds
    GRIND_EXPECTED_FINAL = Point(1.4720, -0.1407, -6.7400)

    # call Grind action asynchronously
    grind_result = self._test_action(
      'Grind',
      ow_lander.msg.GrindAction,
      ow_lander.msg.GrindGoal(),
      GRIND_MAX_DURATION,
      self._assert_regolith_not_present,
      expected_final = GRIND_EXPECTED_FINAL,
      x_start         = 1.65,
      y_start         = 0.0,
      depth           = 0.15, # grind deep so terrain is modified
      length          = 0.7,
      parallel        = True,
      ground_position = GROUND_POSITION
    )

  """
  Tests a default dig linear action and asserts regolith does spawn during the
  operation and remains in the scoop until the end of it.
  """
  def test_03_dig_linear(self):

    DIG_LINEAR_MAX_DURATION = 110.0
    DIG_LINEAR_EXPECTED_FINAL = Point(2.2404, -0.0121, -7.0493)

    # call Grind action asynchronously
    dig_linear_result = self._test_action(
      'DigLinear',
      ow_lander.msg.DigLinearAction,
      ow_lander.msg.DigLinearGoal(),
      DIG_LINEAR_MAX_DURATION,
      self._assert_scoop_regolith_containment,
      expected_final = DIG_LINEAR_EXPECTED_FINAL,
      x_start         = 1.46,
      y_start         = 0.0,
      depth           = 0.01,
      length          = 0.1,
      ground_position = GROUND_POSITION
    )

    # verify regolith models spawned
    self._assert_regolith_present()

  """
  Discards material over the default discard position and asserts it leaves
  scoop and is cleaned up shortly after hitting the terrain.
  """
  def test_04_discard(self):

    # DISCARD_MAX_DURATION = 50.0
    # NOTE: As long as discard and deliver use RRT*, the additional planning
    #       time must be accounted for, so we use a large interval here
    DISCARD_MAX_DURATION = 200.0
    DISCARD_EXPECTED_FINAL = Point(1.5024, 0.8643, -6.6408)

    discard_result = self._test_action(
      'Discard',
      ow_lander.msg.DiscardAction,
      ow_lander.msg.DiscardGoal(),
      DISCARD_MAX_DURATION,
      self._assert_regolith_transports_and_discards,
      expected_final = DISCARD_EXPECTED_FINAL,
      discard = DISCARD_POSITION
    )

    # assert regolith fell out of scoop following the discard action
    self._assert_scoop_regolith_containment(False)

    # assert regolith was cleaned up after contacting with the terrain
    rospy.sleep(REGOLITH_CLEANUP_DELAY)
    self._assert_regolith_not_present()

  """
  Dig for more material so we can then test delivery into the sample dock. This
  digs non-parallel so that material is produced.
  """
  def test_05_dig_circular(self):

    DIG_CIRCULAR_MAX_DURATION = 60.0
    DIG_CIRCULAR_EXPECTED_FINAL = Point(1.6499, -0.1219, -7.3220)

    dig_circular_result = self._test_action(
      'DigCircular',
      ow_lander.msg.DigCircularAction,
      ow_lander.msg.DigCircularGoal(),
      DIG_CIRCULAR_MAX_DURATION,
      self._assert_scoop_regolith_containment,
      expected_final = DIG_CIRCULAR_EXPECTED_FINAL,
      x_start         = 1.65,
      y_start         = 0.0,
      depth           = 0.01,
      parallel        = False,
      ground_position = GROUND_POSITION
    )

    self._assert_regolith_present()

  """
  Tests a deliver action and asserts that regolith remains in the scoop until
  the final portion when it is dumped into the sample dock. Upon completion of
  the operation it also asserts that regolith models are deleted.
  """
  @unittest.expectedFailure
  def test_06_deliver(self):

    # DELIVER_MAX_DURATION = 60.0
    # NOTE: As long as discard and deliver use RRT*, the additional planning
    #       time must be accounted for, so we use a large interval here
    DELIVER_MAX_DURATION = 200.0
    DELIVER_EXPECTED_FINAL = Point(0.5562, -0.2135, -6.3511)

    deliver_result = self._test_action(
      'Deliver',
      ow_lander.msg.DeliverAction,
      ow_lander.msg.DeliverGoal(),
      DELIVER_MAX_DURATION,
      self._assert_regolith_transports_and_delivers_to_dock,
      expected_final = DELIVER_EXPECTED_FINAL,
    )

    # verify regolith has fallen out of the scoop
    self._assert_scoop_regolith_containment(False)
    self._assert_dock_regolith_containment()
  """
  Test the unstow then stow action.
  Calling this after an optertion is standard operating procedure.
  """
  def test_07_stow(self):
    
    # unstow must come before stow
    self.test_01_unstow()

    STOW_MAX_DURATION = 30.0
    STOW_EXPECTED_FINAL = Point(0.7071, -0.4770, -6.5930)

    stow_result = self._test_action(
      'Stow',
      ow_lander.msg.StowAction,
      ow_lander.msg.StowGoal(),
      STOW_MAX_DURATION,
      self._assert_nothing,
      expected_final = STOW_EXPECTED_FINAL
    )

  """
  Test the ingest sample action. Upon completion, ensure no sample remains in
  the simulation.
  """
  @unittest.expectedFailure
  def test_08_ingest_sample(self):

    INGEST_DURATION = 10

    ingest_result = self._test_action(
      'DockIngestSampleAction',
      ow_lander.msg.DockIngestSampleAction,
      ow_lander.msg.DockIngestSampleGoal(),
      INGEST_DURATION,
      self._assert_nothing
    )

    rospy.sleep(REGOLITH_CLEANUP_DELAY)
    self._assert_regolith_not_present(
      "Regolith remains after sample dock ingest action! It was either\n"
      "not properly delivered into the sample dock or the ingest sample\n"
      "action failed."
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'terrain_interaction', TerrainInteraction)
