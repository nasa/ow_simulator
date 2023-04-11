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
import moveit_commander
from abc import ABC, abstractmethod
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from tf2_geometry_msgs import do_transform_pose

from ow_lander import constants
from ow_lander import math3d
from ow_lander.common import radians_equivalent, in_closed_range, create_header
from ow_lander.subscribers import LinkStateSubscriber, JointAnglesSubscriber
from ow_lander.arm_interface import OWArmInterface
from ow_lander.frame_transformer import FrameTransformer
from ow_lander.trajectory_sequence import TrajectorySequence, PlanningException

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
    # initialize/reference
    self._arm = OWArmInterface()
    # initialize interface for querying scoop tip position
    self._arm_tip_monitor = LinkStateSubscriber('lander::l_scoop_tip')
    self._start_server()


class ArmTrajectoryMixin(ArmActionMixin, ABC):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  def execute_action(self, goal):
    try:
      self._arm.checkout_arm(self.name)
      plan = self.plan_trajectory(goal)
      self._arm.execute_arm_trajectory(plan,
        action_feedback_cb = self.publish_feedback_cb)
    except (RuntimeError, PlanningException) as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err))
    else:
      self._arm.checkin_arm(self.name)
      self._set_succeeded(f"{self.name} trajectory succeeded")

  def publish_feedback_cb(self):
    """Publishes the action's feedback. Most arm actions publish the scoop tip
    position under the name "current" in their feedback, so that is what this
    method does by default, but it can be overridden by the child class.
    """
    pass

  @abstractmethod
  def plan_trajectory(self, goal):
    pass


class GrinderTrajectoryMixin(ArmTrajectoryMixin):

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
      self._arm.execute_arm_trajectory(plan)
    except (RuntimeError, PlanningException) as err:
      self._cleanup()
      self._set_aborted(str(err))
    else:
      self._cleanup()
      self._set_succeeded(f"{self.name} trajectory succeeded")


class FrameMixin:
  """Can be inherited by an arm action that operates in different frames.
  THIS CLASS DOES NOT implement an arm interface
    This must be inherited along with ArmActionMixin or one of its children. FrameMixin should come before ArmActionMixin in the inheritance order.
  THIS CLASS DOES NOT support the grinder move group
    All transforms, trajectory planning, and pose queries are done using only
    the 'arm' move group, and there is no way to select the 'grinder' move
    group while using this class' methods.
  """

  COMPARISON_FRAME = 'world'

  @classmethod
  def get_comparison_transform(cls, source_frame):
    transform = FrameTransformer().lookup_transform(cls.COMPARISON_FRAME,
                                                    source_frame)
    if transform is None:
      raise RuntimeError("Failed to acquire transform")
    return transform

  @classmethod
  def poses_equivalent(cls, pose1, pose2):
    return math3d.poses_approx_equivalent(pose1, pose2,
      constants.ARM_POSE_METER_TOLERANCE, constants.ARM_POSE_RADIAN_TOLERANCE)

  @classmethod
  def get_frame_id_from_index(cls, frame):
    if frame not in constants.FRAME_ID_MAP:
      raise RuntimeError(f"Unrecognized frame index {frame}. "
                         f"Options are {str(constants.FRAME_ID_MAP)}")
    # selecting relative is the same as selecting the Tool frame and vice versa
    return constants.FRAME_ID_MAP[frame]

  @classmethod
  def validate_normalization(cls, x):
    """Normalize a vector/quaternion, or reject if all elements are zero.
    x -- Vector3 or Quaternion
    returns normalized form of x or raise an error if all elements of x are zero
    """
    is_quaternion = hasattr(x, 'w')
    dp = math3d.dot(x, x)
    if dp == 0:
      INVALID_ERROR = "{meaning} is the zero-{geometry} and cannot represent " \
                      "a {represents}"
      if is_quaternion:
        raise RuntimeError(
          INVALID_ERROR.format(
            meaning='Orientation', geometry='quaternion', represents='rotation')
        )
      else:
        raise RuntimeError(
          INVALID_ERROR.format(
            meaning='Normal', geometry='vector', represents='direction')
        )
      return None
    elif dp != 1.0:
      n = math3d.normalize(x)
      MODIFIED_WARN = "{meaning} is not normalized and will be interpreted " \
                      "as {value} instead of the provided value"
      if is_quaternion:
        rospy.logwarn(MODIFIED_WARN.format(
          meaning='Orientation', value=f"({n.x}, {n.y}, {n.z}, {n.w})"))
      else:
        rospy.logwarn(MODIFIED_WARN.format(
          meaning='Normal', value=f"({n.x}, {n.y}, {n.z})"))
      return n
    else:
      return x

  def __init__(self, end_effector = 'l_scoop_link', *args, **kwargs):
    self._end_effector = end_effector
    super().__init__(*args, **kwargs)

  def get_intended_position(self, frame_index, move_relative, position):
    frame_id = self.get_frame_id_from_index(frame_index)
    intended_position = position
    if move_relative:
      # treat as additive to the current pose
      current = self.get_end_effector_pose(frame_id)
      if current == None:
        raise RuntimeError("Failed to query current end-effector position in "
                           f"the {frame_id} frame. Cannot perform relative "
                           "motion.")
      intended_position = math3d.add(current.pose.position, position)
    return PointStamped(header=create_header(frame_id),
                        point=intended_position)

  def get_intended_pose(self, frame_index, move_relative, pose):
    frame_id = self.get_frame_id_from_index(frame_index)
    position = pose.position
    orientation = self.validate_normalization(pose.orientation)
    intended_pose = Pose(position, orientation)
    if move_relative:
      # treat as additive to the current pose
      current = self.get_end_effector_pose(frame_id)
      if current == None:
        raise RuntimeError("Failed to query current end-effector pose in the"
                           f"{frame_id} frame. Cannot perform relative motion.")
      intended_pose = Pose(
        math3d.add(current.pose.position, position),
        math3d.quaternion_multiply(current.pose.orientation, orientation)
      )
    return PoseStamped(header=create_header(frame_id),
                       pose=intended_pose)

  def transform_to_planning_frame(self, intended_geometry):
    planning_frame = self._arm.move_group_scoop.get_pose_reference_frame()
    point = FrameTransformer().transform(intended_geometry, planning_frame)
    if point is None:
      raise PlanningException(
        f"Failed to transform requested {type(intended_geometry)} from "
        f"{intended_geometry.frame_id} to the {planning_frame} frame"
      )
    return point

  def verify_pose_reached(self, intended_pose, transform):
    expected = do_transform_pose(intended_pose, transform)
    # NOTE: When checking if a pose has been reached following a movement, it's
    # safest to wait for the next transform to become available in case there is
    # residual movement
    actual = self.get_end_effector_pose(
      self.COMPARISON_FRAME, rospy.Time.now(), rospy.Duration(1.0))
    return self.poses_equivalent(expected.pose, actual.pose)

  def get_end_effector_pose(self, frame_id, timestamp=rospy.Time(0),
                            timeout=rospy.Duration(0)):
    """Look up the pose of the set end-effector
    frame_id     -- Frame ID in which to provide the result
    timestamp    -- See method comments in frame_transformer.py for usage
                    default: rospy.Time(0)
    timeout      -- See method comments in frame_tansformer.py for usage
                    default: rospy.Duration(0)
    returns geometry_msgs.PoseStamped
    """
    pose = self._arm.move_group_scoop.get_current_pose(self._end_effector)
    pose.header.stamp = timestamp
    return FrameTransformer().transform(pose, frame_id, timeout)

  def plan_end_effector_to_pose(self, pose):
    """Plan a trajectory from arm's current pose to a new pose
    pose         -- Stamped pose plan will place end-effector at
    end_effector -- Name of end_effector
    """
    pose_t = self.transform_to_planning_frame(pose)
    # plan trajectory to pose in the arm's pose frame
    sequence = TrajectorySequence(
      self._arm.robot, self._arm.move_group_scoop, self._end_effector)
    sequence.plan_to_pose(pose_t.pose)
    return sequence.merge()


class ModifyJointValuesMixin(ArmActionMixin, ABC):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._arm_joints_monitor = JointAnglesSubscriber(constants.ARM_JOINTS)

  def angles_reached(self, target_angles):
    actual = self._arm_joints_monitor.get_joint_positions()
    return all(
      [
        radians_equivalent(a, b, constants.ARM_JOINT_TOLERANCE)
          for a, b in zip(actual, target_angles)
      ]
    )

  def publish_feedback_cb(self):
    self._publish_feedback(
      angles=self._arm_joints_monitor.get_joint_positions())

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
        final_angles=self._arm_joints_monitor.get_joint_positions())
    else:
      self._arm.checkin_arm(self.name)
      if self.angles_reached(new_positions):
        self._set_succeeded("Arm joints moved successfully",
          final_angles=self._arm_joints_monitor.get_joint_positions())
      else:
        self._set_aborted("Arm joints failed to reach intended target angles",
          final_angles=self._arm_joints_monitor.get_joint_positions())

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
    if not in_closed_range(pan, constants.PAN_MIN, constants.PAN_MAX):
      raise RuntimeError(f"Requested pan {pan} is not within allowed limits.")
    if not in_closed_range(tilt, constants.TILT_MIN, constants.TILT_MAX):
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

  # NOTE: the following move_pan and move_tilt functions have been
  # dumbly factored out of the previous function.  The FIXME comments
  # above have been omitted but apply.  A more refined refactoring is
  # left to a future iteration.

  def move_pan(self, pan):
    if not in_closed_range(pan, constants.PAN_MIN, constants.PAN_MAX):
      raise RuntimeError(f"Requested pan {pan} is not within allowed limits.")

    # publish requested values to start pan trajectory
    self._pan_pub.publish(pan)

    FREQUENCY = 1 # Hz
    TIMEOUT = 60 # seconds
    rate = rospy.Rate(FREQUENCY)
    for i in range(0, int(TIMEOUT * FREQUENCY)):
      if self._is_preempt_requested():
        return False
      # publish feedback message
      self.publish_feedback_cb()
      # check if joints have arrived at their goal values
      if radians_equivalent(pan, self._pan_pos, constants.PAN_TOLERANCE):
        return True
      rate.sleep()
    raise RuntimeError("Timed out waiting for pan value to reach goal.")

  def move_tilt(self, tilt):
    if not in_closed_range(tilt, constants.TILT_MIN, constants.TILT_MAX):
      raise RuntimeError(f"Requested tilt {tilt} is not within allowed limits.")

    # publish requested values to start tilt trajectory
    self._tilt_pub.publish(tilt)

    FREQUENCY = 1 # Hz
    TIMEOUT = 60 # seconds
    rate = rospy.Rate(FREQUENCY)
    for i in range(0, int(TIMEOUT * FREQUENCY)):
      if self._is_preempt_requested():
        return False
      # publish feedback message
      self.publish_feedback_cb()
      # check if joints have arrived at their goal values
      if radians_equivalent(tilt, self._tilt_pos, constants.TILT_TOLERANCE):
        return True
      rate.sleep()
    raise RuntimeError("Timed out waiting for tilt value to reach goal.")

  def publish_feedback_cb(self):
    """overrideable"""
    pass
