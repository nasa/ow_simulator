#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest

import ow_lander.msg
from ow_regolith.srv import SpawnRegolith
from geometry_msgs.msg import Point

from ow_sim_test import action_testing

PKG = 'ow_sim_tests'
TEST_NAME = 'arm_unstow'
roslib.load_manifest(PKG)

class TestArmUnstow(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_unstow_test")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def spawn_regolith(self, position, frame):
    SERVICE = '/ow_regolith/spawn_regolith'
    rospy.wait_for_service(SERVICE)
    spawn = rospy.ServiceProxy(SERVICE, SpawnRegolith)
    try:
      result = spawn(position, frame)
    except rospy.ServiceException as err:
      rospy.logwarn(f"Failed to spawn regolith")

  def test_01_dock_ingest_sample_empty(self):
    dock_ingest_sample_result = test_arm_action(self,
      'DockIngestSample', ow_lander.msg.DockIngestSampleAction,
      ow_lander.msg.DockIngestSampleGoal(),
      TEST_NAME, "test_01_dock_ingest_sample_empty",
      server_timeout = 50.0 # (seconds) first action call needs longer timeout
    )

  def test_02_dock_ingest_sample_full(self):
    # fill sample dock with regolith
    X_INC = 0.01
    rate = rospy.Rate(2)
    for i in range(10):
      offset = Point(i * X_INC if i % 2 == 0 else -X_INC, -0.4, 0)
      self.spawn_regolith(offset, "lander::lander_sample_dock_link")
      rate.sleep()

    dock_ingest_sample_result = test_arm_action(self,
      'DockIngestSample', ow_lander.msg.DockIngestSampleAction,
      ow_lander.msg.DockIngestSampleGoal(),
      TEST_NAME, "test_01_dock_ingest_sample_full"
    )

    

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmUnstow)
