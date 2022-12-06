# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_actions.server import ActionServerBase
from ow_actions.arm import ArmActionMixin

class GrindServer(ArmActionMixin, ActionServerBase):

  name          = 'Grind'
  action_type   = ow_lander.msg.GrindAction
  goal_type     = ow_lander.msg.GrindGoal
  feedback_type = ow_lander.msg.GrindFeedback
  result_type   = ow_lander.msg.GrindResult

  def __init__(self):
    super().__init__()
    self._start_server()

  def execute_action(self, goal):

    plan = self._planner.grind(goal)

    self._switch_to_grind_controller()

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

    self._switch_to_arm_controller()

