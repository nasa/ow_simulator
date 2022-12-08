# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_lander.server import ActionServerBase
from ow_lander.arm import ArmActionMixin

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
    try:
      ArmActionMixin._checkout_arm()
      self._switch_to_grinder_controller()
      plan = self._planner.grind(goal)
      self._execute_arm_trajectory(plan)
    except RuntimeError as err:
      self._set_aborted(str(err), final=self._get_arm_tip_position())
    else:
      self._set_succeeded("Arm trajectory succeeded",
        final=self._get_arm_tip_position())
    finally:
      self._switch_to_arm_controller()
      ArmActionMixin._checkin_arm()
