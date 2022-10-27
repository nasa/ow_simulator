#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import actionlib

import ow_lander.msg
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import PointCloud2

class CameraCaptureActionServer(object):
  def __init__(self, name):
    self._action_name = name
    self._exposure_pub = rospy.Publisher(
      '/gazebo/plugins/camera_sim/exposure', Float64, queue_size=10)
    self._trigger_pub = rospy.Publisher(
      '/StereoCamera/left/image_trigger', Empty, queue_size=10)
    self._point_cloud_sub = rospy.Subscriber('/StereoCamera/points2',
                                             PointCloud2, self._handle_point_cloud)
    self._server = actionlib.SimpleActionServer(self._action_name,
                                                ow_lander.msg.CameraCaptureAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)
    self._server.start()

  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    self.point_cloud_created = False

    # Set exposure if it is specified
    if goal.exposure > 0:
      self._exposure_pub.publish(goal.exposure)
      # There is no guarantee that exposure is set before the image is triggered.
      # Pause to make it highly likely that exposure is received first.
      r.sleep()

    self._trigger_pub.publish()

    for i in range(0, 5):
      if self._server.is_preempt_requested():
        rospy.loginfo(f'{self._action_name}: Preempted')
        self._server.set_preempted()
        return
      if self.point_cloud_created:
        rospy.loginfo(f'{self._action_name}: Succeeded')
        self._server.set_succeeded('point cloud received')
        return
      r.sleep()

    rospy.loginfo(f'{self._action_name}: Failed')
    self._server.set_aborted('timed out')

  def _handle_point_cloud(self, points):
    """
    :type points: sensor_msgs.msg.PointCloud2
    """
    # CameraCapture was successful if a point cloud is received. However, there
    # does not appear to be a way to associate this point cloud with the
    # original trigger message. A trigger could have been sent without using
    # the CameraCapture action client.
    self.point_cloud_created = True

if __name__ == '__main__':
    rospy.init_node('camera_capture_client_py')
    server = CameraCaptureActionServer('CameraCapture')
    rospy.spin()
