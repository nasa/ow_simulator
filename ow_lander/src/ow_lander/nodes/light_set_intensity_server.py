#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander.ow_action_server import OWActionServer

import ow_lander.msg

from irg_gazebo_plugins.msg import ShaderParamUpdate

class LightSetIntensityServer(OWActionServer):

  def __init__(self):
    super(LightSetIntensityServer, self).__init__(
      'LightSetIntensity',
      LightSetIntensityAction,
      LightSetIntensityGoal,
      LightSetIntensityFeedback,
      LightSetIntensityResult
    )

    # set up interface for changing mast light brightness
    self.light_pub = rospy.Publisher('/gazebo/global_shader_param',
                                     ShaderParamUpdate,
                                     queue_size=1)
    self.light_msg = ShaderParamUpdate()
    self.light_msg.shaderType = ShaderParamUpdate.SHADER_TYPE_FRAGMENT

    self.start_server()

  def _on_action_called(self, goal):
    self.result.success, self.result.message = self._set_light_intensity(
      goal.name.lower(),
      goal.intensity
    )
    if self.result.success:
      self.set_succeded()
    else:
      self.set_aborted()

  def _set_light_intensity(self, name, intensity):
    # check intensity range
    if intensity < 0.0 or intensity > 1.0:
      return False, f"Light intensity setting failed. " \
                    f"Intensity = {intensity} is out of range."
    if name == 'left':
      self.light_msg.paramName = 'spotlightIntensityScale[0]'
    elif name == 'right':
      self.light_msg.paramName = 'spotlightIntensityScale[1]'
    else:
      return False, f"Light intensity setting failed. " \
                    f"\'{name}\' is not a light indentifier."
    self.light_msg.paramValue = str(intensity)
    self.light_pub.publish(self.light_msg)
    return True, f"{name} light intensity setting succeeded."

if __name__ == '__main__':
  rospy.init_node('light_set_intensity_server')
  server = LightSetIntensityServer()
  rospy.spin()
