# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_actions.server import ActionServerBase

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

  def execute(self, goal):
    result = self._set_light_intensity(goal.name.lower(), goal.intensity)
    if result.success:
      self._set_succeeded(result)
    else:
      self._set_aborted(result)

  def _set_light_intensity(self, name, intensity):
    # check intensity range
    if intensity < 0.0 or intensity > 1.0:
      return self.result_type(
        False, f"Light intensity setting failed. Intensity = {intensity} is " \
               f"out of range."
      )
    if name == 'left':
      self.light_msg.paramName = 'spotlightIntensityScale[0]'
    elif name == 'right':
      self.light_msg.paramName = 'spotlightIntensityScale[1]'
    else:
      return self.result_type(
        False, f"Light intensity setting failed. \'{name}\' is not a light "
               f"indentifier."
      )
    self.light_msg.paramValue = str(intensity)
    self.light_pub.publish(self.light_msg)
    return self.result_type(True, f"{name} light intensity setting succeeded.")
