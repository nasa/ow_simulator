#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib

import ow_lander.msg
from irg_gazebo_plugins.msg import ShaderParamUpdate

class LightSetIntensityActionServer(object):

  def __init__(self, name):
    # light shader interface setup
    self.light_pub = rospy.Publisher(
      "/gazebo/global_shader_param",
      ShaderParamUpdate,
      queue_size=1
    )
    self.light_msg = ShaderParamUpdate()
    self.light_msg.shaderType = ShaderParamUpdate.SHADER_TYPE_FRAGMENT
    # action server setup
    self._action_name = name
    self._fdbk = ow_lander.msg.LightSetIntensityFeedback()
    self._result = ow_lander.msg.LightSetIntensityResult()
    self._server = actionlib.SimpleActionServer(
      self._action_name,
      ow_lander.msg.LightSetIntensityAction,
      execute_cb=self.on_light_set_intensity_action,
      auto_start=False
    )
    self._server.start()

  def _set_light_intensity(self, goal):
    name = goal.name.lower() # make case insentive
    intensity = goal.intensity
    # check intensity range
    if intensity < 0.0 or intensity > 1.0:
      return False, "Light intensity setting failed. " \
                    "Intensity = %f is out of range." % intensity
    if name == 'left':
      self.light_msg.paramName = 'spotlightIntensityScale[0]'
    elif name == 'right':
      self.light_msg.paramName = 'spotlightIntensityScale[1]'
    else:
      return False, 'Light intensity setting failed. ' \
                    '\'%s\' is not a light indentifier.' % name
    self.light_msg.paramValue = str(intensity)
    self.light_pub.publish(self.light_msg)
    return True, '%s light intensity setting succeeded.' % name

  def on_light_set_intensity_action(self, goal):
    self._result.success, self._result.message = self._set_light_intensity(goal)
    if self._result.success:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._server.set_succeeded(self._result)
    else:
      rospy.loginfo('%s: Failed' % self._action_name)
      self._server.set_aborted(self._result)

if __name__ == '__main__':
  SERVER_NAME = 'LightSetIntensity'
  rospy.init_node(SERVER_NAME)
  server = LightSetIntensityActionServer(SERVER_NAME)
  rospy.spin()
