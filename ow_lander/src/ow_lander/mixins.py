# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines action server mixin classes. If there is need for 2 or more action
servers that perform similar operations, define a shared mixin class for them
here. Presently only arm mixin classes exist in this module, but it can also
define non-arm mixins.
"""

import sys
from abc import ABC, abstractmethod
import rospy
import moveit_commander
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from ow_lander.arm_interface import ArmInterface
from ow_lander.trajectory_planner import ArmTrajectoryPlanner
from ow_lander.subscribers import LinkStateSubscriber, JointAnglesSubscriber
from ow_lander.common import radians_equivalent, in_closed_range
from ow_lander import math3d
from ow_lander.frame_transformer import FrameTransformer
from ow_lander import constants

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

  @abstractmethod
  def plan_trajectory(self, goal):
    pass


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

class ModifyPoseMixin:
  """Can be inherited by ArmMoveActions that modify pose. DOES NOT implement an
  arm interface, and so must be inherited along with ArmActionMixin or one of
  its children.
  """

  ARM_END_EFFECTOR = 'l_scoop_tip'
  COMPARISON_FRAME = 'world'

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.abort_message = ""
    self.old_tool_transform = None

  def handle_frame_goal(self, goal):
    self.abort_message = ""
    self.old_tool_transform = None
    if goal.frame not in constants.FRAME_ID_MAP:
      self.abort_message = f"Unrecognized frame {goal.frame}"
      return None
    # selecting relative is the same as selecting the Tool frame and vice versa
    relative = goal.relative or goal.frame == constants.FRAME_TOOL
    frame_id = constants.FRAME_ID_MAP[constants.FRAME_TOOL] if relative \
               else constants.FRAME_ID_MAP[goal.frame]

    # save tool transform now so the old transform can be used for comparison
    if relative:
      self.old_tool_transform = FrameTransformer() \
        .lookup_transform(self.COMPARISON_FRAME, frame_id)
      if self.old_tool_transform is None:
        self.abort_message = "Failed to lookup TOOL frame transform"
        return None
    return frame_id

  def pose_reached(self, pose):
    # check if requested pose agrees with commanded pose in comparison frame
    final = self._planner.get_end_effector_pose(self.ARM_END_EFFECTOR,
      frame_id=self.COMPARISON_FRAME)
    # the transform before tool movement must be used because the Tool frame
    # moves with the tool
    expected = None
    if self.old_tool_transform is not None: # if movement was relative
      # this function ignores the header
      expected = do_transform_pose(pose,
        self.old_tool_transform)
      self.old_tool_transform = None
    else:
      expected = FrameTransformer().transform(pose, self.COMPARISON_FRAME)
    if final is None or expected is None:
      self.abort_message = "Failed to perform necessary transforms to verify " \
                           "final pose"
      return False
    if math3d.poses_approx_equivalent(expected.pose, final.pose,
        constants.ARM_POSE_METER_TOLERANCE, constants.ARM_POSE_METER_TOLERANCE):
      return True
    else:
      self.abort_message = "Failed to reach commanded pose"

class ModifyJointValuesMixin(ArmActionMixin, ABC):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._arm_joints_monitor = JointAnglesSubscriber(constants.ARM_JOINTS)

  # NOTE: these two helper functions are hacky work-arounds necessary because
  #       ArmMoveJoints.action uses wrong names in feedback and result (OW-1096)
  def __format_result(self):
    return {self.result_type.__slots__[0] : self._arm_joints_monitor
      .get_joint_positions()}
  def __format_feedback(self):
    return {self.feedback_type.__slots__[0] : self._arm_joints_monitor
      .get_joint_positions()}

  def angles_reached(self, target_angles):
    actual = self._arm_joints_monitor.get_joint_positions()
    return all(
      [
        radians_equivalent(a, b, constants.ARM_JOINT_TOLERANCE)
          for a, b in zip(actual, target_angles)
      ]
    )

  def publish_feedback_cb(self):
    self._publish_feedback(**self.__format_feedback())

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
      if self.angles_reached(new_positions):
        self._set_succeeded("Arm joints moved successfully",
          **self.__format_result())
      else:
        self._set_aborted("Arm joints failed to reach intended target angles",
          **self.__format_result())

  @abstractmethod
  def modify_joint_positions(self, goal):
    pass

class PanTiltMoveMixin:

  JOINT_STATES_TOPIC = "/joint_states"

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    ANTENNA_PAN_POS_TOPIC  = '/ant_pan_position_controller/command'
    ANTENNA_TILT_POS_TOPIC = '/ant_tilt_position_controller/command'
    self._pan_pub = rospy.Publisher(
      ANTENNA_PAN_POS_TOPIC, Float64, queue_size=1)
    self._tilt_pub = rospy.Publisher(
      ANTENNA_TILT_POS_TOPIC, Float64, queue_size=1)
    self._subscriber = rospy.Subscriber(
      self.JOINT_STATES_TOPIC, JointState, self._handle_joint_states)
    self._start_server()

  def _handle_joint_states(self, data):
    # position of pan and tilt of the lander is obtained from JointStates
    try:
      self._pan_pos = data.position[constants.JOINT_STATES_MAP["j_ant_pan"]]
      self._tilt_pos = data.position[constants.JOINT_STATES_MAP["j_ant_tilt"]]
    except KeyError as err:
      rospy.logerr_throttle(1,
        f"PanTiltMoveMixin: {err}; joint value missing in "\
        f"{self.JOINT_STATES_TOPIC} topic")
      return

  def move_pan_and_tilt(self, pan, tilt):
    # FIXME: tolerance should not be necessary once the float precision
    #        problem is fixed by command unification (OW-1085)
    if not in_closed_range(pan, constants.PAN_MIN, constants.PAN_MAX,
                           constants.PAN_TILT_INPUT_TOLERANCE):
      raise RuntimeError(f"Requested pan {pan} is not within allowed limits.")
    if not in_closed_range(tilt, constants.TILT_MIN, constants.TILT_MAX,
                           constants.PAN_TILT_INPUT_TOLERANCE):
      raise RuntimeError(f"Requested tilt {tilt} is not within allowed limits.")

    # publish requested values to start pan/tilt trajectory
    self._pan_pub.publish(pan)
    self._tilt_pub.publish(tilt)

    # FIXME: The outcome of ReferenceMission1 happens to be closely tied to
    #        the value of FREQUENCY. When a fast frequency was selected (10 Hz),
    #        the image used to identify a sample location would be slightly to
    #        the right than the image would have been if the frequency is set to
    #        1 Hz, which would result in a sample location being selected that
    #        is about half a meter closer to the lander than otherwise.
    #        Such a dependency on FREQUENCY should not occur and implies that
    #        the loop terminates before the antenna mast has completed its
    #        movement. This breaks the synchronicity of actions, and therefore
    #        of PLEXIL commands.
    #        The loop break should instead trigger when both antenna joint
    #        velocities are near enough to zero.
    #         This issue is captured by OW-1105
    # loop until pan/tilt reach their goal values
    FREQUENCY = 1 # Hz
    TIMEOUT = 60 # seconds
    rate = rospy.Rate(FREQUENCY)
    for i in range(0, int(TIMEOUT * FREQUENCY)):
      if self._is_preempt_requested():
        return False
      # publish feedback message
      self.publish_feedback_cb()
      # check if joints have arrived at their goal values
      if radians_equivalent(pan, self._pan_pos, constants.PAN_TOLERANCE) and \
          radians_equivalent(tilt, self._tilt_pos, constants.TILT_TOLERANCE):
        return True
      rate.sleep()
    raise RuntimeError("Timed out waiting for pan/tilt values to reach goal.")

  def publish_feedback_cb(self):
    """overrideable"""
    pass

