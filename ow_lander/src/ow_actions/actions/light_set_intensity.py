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
