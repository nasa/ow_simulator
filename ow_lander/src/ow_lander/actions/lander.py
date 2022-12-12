# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_lander.server import ActionServerBase

from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import PointCloud2

from irg_gazebo_plugins.msg import ShaderParamUpdate

class LightSetIntensityServer(ActionServerBase):

  name          = 'LightSetIntensity'
  action_type   = ow_lander.msg.LightSetIntensityAction
  goal_type     = ow_lander.msg.LightSetIntensityGoal
  feedback_type = ow_lander.msg.LightSetIntensityFeedback
  result_type   = ow_lander.msg.LightSetIntensityResult

  def __init__(self):
    super(LightSetIntensityServer, self).__init__()
    # set up interface for changing mast light brightness
    self.light_pub = rospy.Publisher('/gazebo/global_shader_param',
                                     ShaderParamUpdate,
                                     queue_size=1)
    self.light_msg = ShaderParamUpdate()
    self.light_msg.shaderType = ShaderParamUpdate.SHADER_TYPE_FRAGMENT
    self._start_server()

  def execute_action(self, goal):
    intensity = goal.intensity
    name = goal.name.lower()
    # check intensity range
    if intensity < 0.0 or intensity > 1.0:
      msg = f"Intensity = {intensity} is out of range."
      # NOTE: This action's result format is redundant since actionlib already
      #       conveys both a "success" flag and a "message" string. This
      #       redundancy will be removed following command unification.
      self._set_aborted(msg, success=False, message=msg)
      return
    # check for correct names
    if name == 'left':
      self.light_msg.paramName = 'spotlightIntensityScale[0]'
    elif name == 'right':
      self.light_msg.paramName = 'spotlightIntensityScale[1]'
    else:
      msg = f"\'{name}\' is not a light indentifier."
      self._set_aborted(msg, success=False, message=msg)
      return
    self.light_msg.paramValue = str(goal.intensity)
    self.light_pub.publish(self.light_msg)
    msg = f"{name} light intensity set successfully."
    self._set_succeeded(msg, success=True, message=msg)


class CameraCaptureServer(ActionServerBase):

  name          = 'CameraCapture'
  action_type   = ow_lander.msg.CameraCaptureAction
  goal_type     = ow_lander.msg.CameraCaptureGoal
  feedback_type = ow_lander.msg.CameraCaptureFeedback
  result_type   = ow_lander.msg.CameraCaptureResult

  def __init__(self):
    super(CameraCaptureServer, self).__init__()
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
    self._start_server()

  def _handle_point_cloud(self, points):
    """
    :type points: sensor_msgs.msg.PointCloud2
    """
    # CameraCapture was successful if a point cloud is received. However, there
    # does not appear to be a way to associate this point cloud with the
    # original trigger message. A trigger could have been sent without using
    # the CameraCapture action client.
    self.point_cloud_created = True

  def execute_action(self, goal):
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
      # TODO: investigate replacing with an optional preempt callback function
      if self._is_preempt_requested():
        self._set_preempted("Action was preempted")
        return
      if self.point_cloud_created:
        self._set_succeeded("Point cloud received")
        return
      rate.sleep()
    self._set_aborted("Timed out waiting for point cloud")

