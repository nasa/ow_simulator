#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import unittest
from std_msgs.msg import Float32

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)


# a class that monitors minimum reported frame rate by a gazebo simulation
class MonitorMinimumFPS(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node("test_minimum_fps", anonymous=True)
    self.min_expected_fps = rospy.get_param("/minimum_fps/min_fps")
    self.warmup_period = rospy.get_param("/minimum_fps/warmup_period")
    self.test_duration = rospy.get_param("/minimum_fps/test_duration")
    self.assertGreater(self.test_duration, self.warmup_period,
                       "test_duration must be larger than warmup_period")
    self.min_observed_fps = float('inf')
    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def handle_avg_fps(self, avg_fps):

    if rospy.get_time() - self.test_start_time < self.warmup_period:
      rospy.loginfo("fps value: " + str(avg_fps) + " skipped")
      return

    rospy.loginfo("fps value: " + str(avg_fps) + " considered")
    self.min_observed_fps = min(self.min_observed_fps, avg_fps.data)

  def test_minimum_fps(self):

    self.test_start_time = rospy.get_time()

    self.avg_fps_sub = rospy.Subscriber(
        '/fps_monitor/avg_fps', Float32, self.handle_avg_fps)

    elapsed = 0
    while not rospy.is_shutdown() and elapsed < self.test_duration:
      elapsed = rospy.get_time() - self.test_start_time
      rospy.sleep(0.1)

    self.assertLess(self.min_observed_fps, float('inf'),
                    "no intel on fps was received!!!!")

    self.assertGreater(self.min_observed_fps, self.min_expected_fps,
                       "observed fps: {} fell below the lowest threshold: {}".format(
                        round(self.min_observed_fps), round(self.min_expected_fps)))


if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'test_minimum_fps', MonitorMinimumFPS)
