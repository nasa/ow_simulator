#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest
from geometry_msgs.msg import Point, WrenchStamped

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)


# a class that monitors minimum reported frame rate by a gazebo simulation
class FT_Sensor_Check(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node("ft_sensor_check", anonymous=True)
    self._test_duration = rospy.get_param("/ft_sensor_check/test_duration")
    self._wrench_msg = None

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def _ft_sensor_dist_pitch_callback(self, msg):
    self._wrench_msg = msg.wrench

  def test_01_check_ft_sensor_stowed(self):

    self._dist_pitch_ft_sensor_sub = rospy.Subscriber(
      "/ft_sensor_dist_pitch", WrenchStamped, self._ft_sensor_dist_pitch_callback)

    test_start_time = rospy.get_time()
    elapsed = 0
    while not rospy.is_shutdown() and elapsed < self._test_duration:
      elapsed = rospy.get_time() - test_start_time
      rospy.sleep(0.1)

    self.assertIsNotNone(self._wrench_msg, "no ft sensor message was received")
    self.assertAlmostEqual(self._wrench_msg.force.y, -14.74, 0)

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'ft_sensor_check', FT_Sensor_Check)
