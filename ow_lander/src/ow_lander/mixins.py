# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines action server mixin classes. If there is need for 2 or more action
servers that perform similar operations, define a shared mixin class for them
here. Presently only arm mixin classes exist in this module, but it can also
define non-arm mixins.
"""

import sys
import rospy
import actionlib
import moveit_commander

from ow_lander.arm_interface import ArmInterface
from ow_lander.trajectory_planner import ArmTrajectoryPlanner
from ow_lander.subscribers import LinkStateSubscriber, JointAnglesSubscriber
from ow_lander.common import radians_equivalent
from ow_lander.constants import ARM_JOINT_TOLERANCE

from abc import ABC, abstractmethod

class ArmActionMixin:
  """Enables an action server to control the OceanWATERS arm. This or one of its
  children classes must be placed first in the inheritance statement.
  e.g.
  class FooArmActionServer(ArmActionMixin, ActionServerBase):
    ...
  """
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    # initialize moveit interface for arm control
    moveit_commander.roscpp_initialize(sys.argv)
    # initialize/reference trajectory planner singleton (for external use)
    self._planner = ArmTrajectoryPlanner()
    # initialize/reference
    self._arm = ArmInterface()
    # initialize interface for querying scoop tip position
    self._arm_tip_monitor = LinkStateSubscriber('lander::l_scoop_tip')
    self._start_server()

  def publish_feedback_cb(self):
    """Publishes the action's feedback. Most arm actions publish the scoop tip
    position under the name "current" in their feedback, so that is what this
    method does by default, but it can be overridden by the child class.
    """
    self._publish_feedback(current=self._arm_tip_monitor.get_link_position())


class ArmTrajectoryMixin(ArmActionMixin, ABC):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  def execute_action(self, goal):
    try:
      self._arm.checkout_arm(self.name)
      plan = self.plan_trajectory(goal)
      self._arm.execute_arm_trajectory(plan)
    except RuntimeError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err))
    else:
      self._arm.checkin_arm(self.name)
      self._set_succeeded("Arm trajectory succeeded")


# DEPRECTATED: This version that provides current and final positions will be
#              removed as a result of command unification. For new or
#              transitioned arm trajectory actions, use ArmTrajectoryMixin
#              instead
class ArmTrajectoryMixinOld(ArmActionMixin, ABC):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  def execute_action(self, goal):
    try:
      self._arm.checkout_arm(self.name)
      plan = self.plan_trajectory(goal)
      self._arm.execute_arm_trajectory(plan,
        action_feedback_cb=self.publish_feedback_cb)
    except RuntimeError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        final=self._arm_tip_monitor.get_link_position())
    else:
      self._arm.checkin_arm(self.name)
      self._set_succeeded(f"{self.name} trajectory succeeded",
        final=self._arm_tip_monitor.get_link_position())

  @abstractmethod
  def plan_trajectory(self, goal):
    pass


class GrinderTrajectoryMixin(ArmActionMixin, ABC):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  def _cleanup(self):
    self._arm.switch_to_arm_controller()
    self._arm.checkin_arm(self.name)

  def execute_action(self, goal):
    try:
      self._arm.checkout_arm(self.name)
      self._arm.switch_to_grinder_controller()
      plan = self.plan_trajectory(goal)
      self._arm.execute_arm_trajectory(plan,
        action_feedback_cb=self.publish_feedback_cb)
    except RuntimeError as err:
      self._cleanup()
      self._set_aborted(str(err),
        final=self._arm_tip_monitor.get_link_position())
    else:
      self._cleanup()
      self._set_succeeded(f"{self.name} trajectory succeeded",
        final=self._arm_tip_monitor.get_link_position())

  @abstractmethod
  def plan_trajectory(self, goal):
    pass


class ModifyJointValuesMixin(ArmActionMixin, ABC):

  def __init__(self, *args, **kwargs):
    ARM_JOINTS = [
      'j_shou_yaw','j_shou_pitch','j_prox_pitch',
      'j_dist_pitch','j_hand_yaw', 'j_scoop_yaw'
    ]
    super().__init__(*args, **kwargs)
    self._arm_joints_monitor = JointAnglesSubscriber(ARM_JOINTS)

  # NOTE: these two helper functions are hacky work-arounds necessary because
  #       ArmMoveJoints.action uses wrong names in feedback and result (OW-1096)
  def __format_result(self, value):
    return {self.result_type.__slots__[0] : value}
  def __format_feedback(self, value):
    return {self.feedback_type.__slots__[0] : value}

  def __check_success(self, target):
    actual = self._arm_joints_monitor.get_joint_positions()
    success = all(
      [
        radians_equivalent(a, b, ARM_JOINT_TOLERANCE)
          for a, b in zip(actual, target)
      ]
    )
    if success:
      self._set_succeeded("Arm joints moved successfully",
        **self.__format_result(actual))
    else:
      self._set_aborted("Arm joints failed to reach intended target",
        **self.__format_result(actual))

  def publish_feedback_cb(self):
    self._publish_feedback(
      **self.__format_feedback(self._arm_joints_monitor.get_joint_positions())
    )

  def execute_action(self, goal):
    try:
      self._arm.checkout_arm(self.name)
      new_positions = self.modify_joint_positions(goal)
      plan = self._planner.plan_arm_to_joint_angles(new_positions)
      self._arm.execute_arm_trajectory(plan,
        action_feedback_cb=self.publish_feedback_cb)
    except (RuntimeError, moveit_commander.exception.MoveItCommanderException) \
        as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        **self.__format_result(self._arm_joints_monitor.get_joint_positions()))
    else:
      self._arm.checkin_arm(self.name)
      self.__check_success(new_positions)

  @abstractmethod
  def modify_joint_positions(self, goal):
    pass
