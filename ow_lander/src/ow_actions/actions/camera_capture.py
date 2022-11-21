#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_actions.server import ActionServerBase
from ow_actions.client import ActionClient

from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import PointCloud2

class CameraCaptureServer(ActionServerBase):

  def __init__(self):
    super(CameraCaptureServer, self).__init__(
      'CameraCapture',
      ow_lander.msg.CameraCaptureAction,
      ow_lander.msg.CameraCaptureGoal(),
      ow_lander.msg.CameraCaptureFeedback(),
      ow_lander.msg.CameraCaptureResult()
    )

    # set up interface for capturing a photograph witht he camera
    self._pub_exposure = rospy.Publisher('/gazebo/plugins/camera_sim/exposure',
                                         Float64,
                                         queue_size=10)
    self._pub_trigger = rospy.Publisher('/StereoCamera/left/image_trigger',
                                        Empty,
                                        queue_size=10)
    self._sub_point_cloud = rospy.Subscriber('/StereoCamera/points2',
                                             PointCloud2,
                                             self._handle_point_cloud)
    self.point_cloud_created = False

    self.start_server()

  def _handle_point_cloud(self, points):
    """
    :type points: sensor_msgs.msg.PointCloud2
    """
    # CameraCapture was successful if a point cloud is received. However, there
    # does not appear to be a way to associate this point cloud with the
    # original trigger message. A trigger could have been sent without using
    # the CameraCapture action client.
    self.point_cloud_created = True

  def _on_action_called(self, goal):
    self.point_cloud_created = False
    # Set exposure if it is specified
    if goal.exposure > 0:
      self._pub_exposure.publish(goal.exposure)
      # There is no guarantee that exposure is set before the image is triggered
      # Pause to make it highly likely that exposure is received first.
      rospy.sleep(1)

    self._pub_trigger.publish()

    # await point cloud or action preempt
    frequency = 10 # Hz
    time_out = 5   # seconds
    rate = rospy.Rate(frequency)
    for i in range(0, int(time_out * frequency)):
      if self._server.is_preempt_requested():
        self.set_preempted()
        return
      if self.point_cloud_created:
        self.set_succeeded("Point cloud received")
        return
      rate.sleep()
    self.set_aborted("Timed out waiting for point cloud")

def spin_action_server():
  rospy.init_node('camera_capture_server')
  server = CameraCaptureServer()
  rospy.spin()

def call_action(exposure):
  client = ActionClient(
    'CameraCapture',
    ow_lander.msg.CameraCaptureAction
  )
  return client.call(
    ow_lander.msg.CameraCaptureGoal(exposure=exposure)
  )
