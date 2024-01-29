# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines a fault messages interface to the OceanWATERS simulator. Integrates
communication mechanism between ow_faults_detection package and the ow_lander package.
"""

import rospy
from ow_lander.common import Singleton
from ow_lander.msg import ActionGoalStatus
from owl_msgs.msg import ArmFaultsStatus
from actionlib_msgs.msg import GoalStatus

class SystemFaultsPublisher(metaclass = Singleton):

  ACTION_GOAL_STATUS_TOPIC = "action_goal_status"

  def __init__(self):
    self._status = [
      GoalStatus() for _ in range(ActionGoalStatus.NUM_GOAL_TYPES)
    ]
    self._goal_state_pub = rospy.Publisher(
      self.ACTION_GOAL_STATUS_TOPIC, ActionGoalStatus,
      queue_size = 1, latch = True
    )

  def publish(self, group_id, status):
    if group_id is None:
      return
    # update internal status list
    timestamp = rospy.Time.now()
    goal_id = str(group_id) # ActionGoalStatus emum value to string
    self._status[group_id].goal_id.stamp = timestamp
    self._status[group_id].goal_id.id    = goal_id
    self._status[group_id].status        = status
    # publish status
    msg = ActionGoalStatus()
    msg.header.stamp    = timestamp
    msg.header.frame_id = 'world'
    msg.status_list     = self._status
    self._goal_state_pub.publish(msg)

  def get_group_status(self, group_id):
    return self._status[group_id].status

  def get_group_str(self, group_id):
    if group_id == ActionGoalStatus.ARM_GOAL:
      return "ARM"
    elif group_id == ActionGoalStatus.TASK_GOAL:
      return "TASK"
    if group_id == ActionGoalStatus.CAMERA_GOAL:
      return "CAMERA"
    if group_id == ActionGoalStatus.PAN_TILT_GOAL:
      return "PAN_TILT"
    else:
      return "UNKNOWN GROUP"


class ArmFaultsPublisher(metaclass = Singleton):

  INTERNAL_ARM_FAULT_TOPIC = "arm_faults_internal"

  def __init__(self):
    self._flags = ArmFaultsStatus.NONE
    self._arm_faults_internal_pub = rospy.Publisher(
      self.INTERNAL_ARM_FAULT_TOPIC, ArmFaultsStatus,
      queue_size = 1, latch = True
    )

  def publish(self, flags):
    self._arm_faults_internal_pub.publish(value = flags)


class SystemFaultsInterface:

  def __init__(self, group_id):
    self._goal_fault_publisher = SystemFaultsPublisher()
    self._group_id = group_id

  def reset_system_faults(self):
    # HACK: The SUCCEEDED state does not reflect the current state of the action
    #   that has been called. It cannot have succeeded yet because it has yet to
    #   be attempted here. This is a hack. ActionGoalStatus.msg is effectively
    #   interpreted by ow_fault_detector as a boolean array because an element
    #   can either be SUCCEEDED or ABORTED. To reset system_faults_status goal
    #   error flags to 0 at the beginning of an action, we broadcast SUCCEEDED.
    self._notify(GoalStatus.SUCCEEDED)

  def notify_succeeded(self):
    self._notify(GoalStatus.SUCCEEDED)

  def notify_aborted(self):
    self._notify(GoalStatus.ABORTED)

  def notify_preempted(self):
    self._notify(GoalStatus.PREEMPTED)

  def is_faulted(self):
    return self._goal_fault_publisher.get_group_status(
      self._group_id) == GoalStatus.ABORTED

  def get_rejected_message(self):
    group = self._goal_fault_publisher.get_group_str(self._group_id)
    return f"Action group, {group}, is faulted. Use the FaultClear action to " \
            "make the group operable again."

  def _notify(self, status):
    self._goal_fault_publisher.publish(self._group_id, status)


class ArmFaultsInterface:
  def __init__(self):
    self._arm_fault_publisher = ArmFaultsPublisher()
    self._flag_values = ArmFaultsStatus.NONE

  def set_arm_faults(self, flags):
    self._flag_values = flags
    self._arm_fault_publisher.publish(self._flag_values)

  def reset_arm_faults(self):
    self._flag_values = ArmFaultsStatus.NONE
    self._arm_fault_publisher.publish(self._flag_values)


class ArmFaultHandler(SystemFaultsInterface, ArmFaultsInterface):
  def __init__(self):
    SystemFaultsInterface.__init__(self, ActionGoalStatus.ARM_GOAL)
    ArmFaultsInterface.__init__(self)


class TaskFaultHandler(SystemFaultsInterface, ArmFaultsInterface):
  def __init__(self):
    SystemFaultsInterface.__init__(self, ActionGoalStatus.TASK_GOAL)
    ArmFaultsInterface.__init__(self)


class CameraFaultHandler(SystemFaultsInterface):
  def __init__(self):
    SystemFaultsInterface.__init__(self, ActionGoalStatus.CAMERA_GOAL)


class PanTiltFaultHandler(SystemFaultsInterface):
  def __init__(self):
    SystemFaultsInterface.__init__(self, ActionGoalStatus.PAN_TILT_GOAL)
