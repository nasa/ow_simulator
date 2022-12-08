# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_lander.server import ActionServerBase
from ow_lander.arm import ArmActionMixin

class StopServer(ArmActionMixin, ActionServerBase):

  name          = 'Stop'
  action_type   = ow_lander.msg.StopAction
  goal_type     = ow_lander.msg.StopGoal
  feedback_type = ow_lander.msg.StopFeedback
  result_type   = ow_lander.msg.StopResult

  def __init__(self):
    super().__init__()
    self._start_server()

  def execute_action(self, _goal):
    if ArmActionMixin._stop_arm():
      self._set_succeeded(
        "Arm trajectory stopped", final=self._get_arm_tip_position())
    else:
      self._set_aborted(
        "No arm trajectory to stop", final=self._get_arm_tip_position())
