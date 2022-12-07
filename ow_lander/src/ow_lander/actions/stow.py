# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_lander.server import ActionServerBase
from ow_lander.arm import ArmActionMixin

class StowServer(ArmActionMixin, ActionServerBase):

  # UNIFICATION TODO: rename "Stow" to "ArmStow"
  name          = 'Stow'
  action_type   = ow_lander.msg.StowAction
  goal_type     = ow_lander.msg.StowGoal
  feedback_type = ow_lander.msg.StowFeedback
  result_type   = ow_lander.msg.StowResult

  def __init__(self):
    super().__init__()
    self._start_server()

  def execute_action(self, _goal):

    ## TODO: is this necessary? can it be redesigned?
    # DEACTIVATED: still investigating how best to incorporate the stop action
    # _server_stop.reset()

    plan = self._planner.plan_arm_to_target('arm_stowed')

    if self._execute_arm_trajectory(plan):
      self._set_succeeded(
        f"{self.name} arm trajectory succeeded",
        final=self._get_arm_tip_position()
      )
    else:
      self._set_aborted(
        f"{self.name} arm trajectory aborted",
        final=self._get_arm_tip_position()
      )
