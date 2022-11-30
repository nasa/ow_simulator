# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_actions.server import ActionServerBase
from ow_actions.arm import ArmActionMixin

class UnstowServer(ArmActionMixin, ActionServerBase):

  # UNIFICATION TODO: rename "Unstow" to "ArmUnstow"
  name          = 'Unstow'
  action_type   = ow_lander.msg.UnstowAction
  goal_type     = ow_lander.msg.UnstowGoal
  feedback_type = ow_lander.msg.UnstowFeedback
  result_type   = ow_lander.msg.UnstowResult

  def __init__(self):
    super().__init__()
    self._start_server()

  def execute_action(self, goal):
    ### TODO: is this necessary? can it be redesigned?
    # _server_stop.reset()

    target_joints = self._arm_trajectory_planner._move_arm.get_named_target_values("arm_unstowed")
    self._arm_trajectory_planner._move_arm.set_joint_value_target(target_joints)
    # NOTE: success, planning_time, and error are not used but are named rather
    #       than dumped for reference
    _success, plan, planning_time, _error = self._arm_trajectory_planner._move_arm.plan()

    self.execute_arm_trajectory(plan)
