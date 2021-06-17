#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest
import time
from std_msgs.msg import Float32

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)


# a class that monitors minimum reported frame rate by a gazebo simulation
class MonitorMinimumFPS(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)

    self.min_expected_fps = rospy.get_param("/minimum_fps/min_fps")
    self.warmup_period = rospy.get_param("/minimum_fps/warmup_period")
    self.test_duration = rospy.get_param("/minimum_fps/test_duration")
    self.assertGreater(self.test_duration, self.warmup_period,
                       "test_duration must be larger than warmup_period")

    self.min_observed_fps = float('inf')

  def handle_avg_fps(self, avg_fps):

    if time.time() - self.test_start_time < self.warmup_period:
      rospy.loginfo("fps value: " + str(avg_fps) + " skipped")
      return

    rospy.loginfo("fps value: " + str(avg_fps) + " considered")
    self.min_observed_fps = min(self.min_observed_fps, avg_fps.data)

  def test_minimum_fps(self):

    rospy.init_node("test_minimum_fps", anonymous=True)

    self.test_start_time = time.time()

    self.avg_fps_sub = rospy.Subscriber(
        '/fps_monitor/avg_fps', Float32, self.handle_avg_fps)

    while not rospy.is_shutdown() and (time.time() - self.test_start_time < self.test_duration):
      time.sleep(0.1)

    self.assertGreater(self.min_observed_fps, self.min_expected_fps)


if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'test_minimum_fps', MonitorMinimumFPS)
