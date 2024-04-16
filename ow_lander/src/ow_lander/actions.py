# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines all lander actions"""

import math
from copy import copy

import rospy
import owl_msgs.msg
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import PointCloud2
from ow_regolith.srv import RemoveRegolith
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3, PoseStamped, Pose
from urdf_parser_py.urdf import URDF
from owl_msgs.msg import ArmFaultsStatus
from gazebo_msgs.msg import LinkStates

import ow_lander.msg
from ow_lander import mixins
from ow_lander import math3d
from ow_lander import faults
from ow_lander import constants
from ow_lander.server import ActionServerBase
from ow_lander.common import normalize_radians, wait_for_subscribers
from ow_lander.exception import (ActionError, ArmError, AntennaError,
                                 ArmPlanningError, ArmExecutionError)
from ow_lander.subscribers import wait_for_message
from ow_lander.power_interface import PowerInterface
from ow_lander.ground_detector import GroundDetector, FTSensorThresholdMonitor
from ow_lander.frame_transformer import FrameTransformer
from ow_lander.trajectory_sequence import TrajectorySequence


# This message is used by both ArmMoveCartesianGuarded and ArmMoveJointsGuarded
NO_THRESHOLD_BREACH_MESSAGE = "Arm failed to reach pose despite neither " \
                              "force nor torque thresholds being breached. " \
                              "Likely the thresholds were set too high or a " \
                              "planning error occurred. Try a lower threshold."

#####################
## ACTION HELPERS
#####################

def _format_guarded_move_success_message(action_name, monitor):
  if monitor.threshold_breached():
    msg = f"{action_name} trajectory stopped by "
    if monitor.force_threshold_breached():
      msg += f"a force of {monitor.get_force():.2f} N"
    if monitor.torque_threshold_breached():
      msg += " and " if monitor.force_threshold_breached() else ""
      msg += f"a torque of {monitor.get_torque():.2f} Nm"
    return msg
  else:
    return f"{action_name} trajectory completed without breaching force or " \
           f"torque thresholds"

def _assert_shou_yaw_in_range(shou_yaw_position):
    """Check if shoulder yaw is within allowable
    shou_yaw_position -- shoulder yaw joint position in radians
    """
    # If shoulder yaw goal angle is out of joint range, abort
    upper = URDF.from_parameter_server().joint_map["j_shou_yaw"].limit.upper
    lower = URDF.from_parameter_server().joint_map["j_shou_yaw"].limit.lower
    if shou_yaw_position <= lower or shou_yaw_position >= upper:
      raise ArmPlanningError("Shoulder yaw is outside of allowable range")

def _compute_workspace_shoulder_yaw(x, y):
    """Compute shoulder yaw angle to trench
    x -- base_link x position
    y -- base_link y position
    """
    yaw = math.atan2(y - constants.Y_SHOU, x - constants.X_SHOU)
    h = math.sqrt(pow(y - constants.Y_SHOU, 2) + pow(x - constants.X_SHOU, 2))
    l = constants.Y_SHOU - constants.HAND_Y_OFFSET
    yaw += math.asin(l / h)
    _assert_shou_yaw_in_range(yaw)
    return yaw
    


#####################
## ARM ACTIONS
#####################

class FaultClearServer(ActionServerBase):

  name            = 'FaultClear'
  action_type     = owl_msgs.msg.FaultClearAction
  goal_type       = owl_msgs.msg.FaultClearGoal
  feedback_type   = owl_msgs.msg.FaultClearFeedback
  result_type     = owl_msgs.msg.FaultClearResult

  def __init__(self):
    super().__init__()
    self._start_server()

  def execute_action(self, goal):
    MSG_FORMAT = "%s goal faults have been cleared."
    if goal.fault == owl_msgs.msg.SystemFaultsStatus.ARM_GOAL_ERROR:
      faults.ArmFaultHandler().reset_system_faults()
      self._set_succeeded(MSG_FORMAT % "ARM")
    elif goal.fault == owl_msgs.msg.SystemFaultsStatus.TASK_GOAL_ERROR:
      faults.TaskFaultHandler().reset_system_faults()
      self._set_succeeded(MSG_FORMAT % "TASK")
    elif goal.fault == owl_msgs.msg.SystemFaultsStatus.CAMERA_GOAL_ERROR:
      faults.CameraFaultHandler().reset_system_faults()
      self._set_succeeded(MSG_FORMAT % "CAMERA")
    elif goal.fault == owl_msgs.msg.SystemFaultsStatus.PAN_TILT_GOAL_ERROR:
      faults.PanTiltFaultHandler().reset_system_faults()
      self._set_succeeded(MSG_FORMAT % "PAN_TILT")
    else:
      self._set_aborted(f"Fault index, {goal.fault}, is not valid.")

class ArmStopServer(mixins.ArmActionMixin, ActionServerBase):

  name          = 'ArmStop'
  action_type   = owl_msgs.msg.ArmStopAction
  goal_type     = owl_msgs.msg.ArmStopGoal
  feedback_type = owl_msgs.msg.ArmStopFeedback
  result_type   = owl_msgs.msg.ArmStopResult

  def execute_action(self, _goal):
    if self._arm.stop_arm():
      self._set_succeeded("Arm trajectory stopped")
    else:
      self._set_aborted("No arm trajectory to stop")


### DEPRECATED: ArmFindSurface should be used in place of GuardedMove
class GuardedMoveServer(mixins.ArmActionMixin, ActionServerBase):

  # NOTE: The "final" in GuardedMove's result is not in the same frame as the
  #       other arm action's finals, which seems misleading from a user
  #       standpoint.

  name          = 'GuardedMove'
  action_type   = ow_lander.msg.GuardedMoveAction
  goal_type     = ow_lander.msg.GuardedMoveGoal
  feedback_type = ow_lander.msg.GuardedMoveFeedback
  result_type   = ow_lander.msg.GuardedMoveResult

  fault_handler = faults.TaskFaultHandler()

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    # TODO: would it make sense to latch this?
    # TODO: is this publisher necessary?
    self._pub_result = rospy.Publisher('/guarded_move_result',
                                       ow_lander.msg.GuardedMoveFinalResult,
                                       queue_size=1)

  def plan_trajectory(self, goal):
    sequence = TrajectorySequence(
      self._arm.robot, self._arm.move_group_scoop, 'l_scoop')
    # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
    targ_elevation = -0.2
    if (goal.start.z+targ_elevation) == 0:
      offset = goal.search_distance
    else:
      offset = (goal.start.z*goal.search_distance)/(goal.start.z+targ_elevation)
    # Compute shoulder yaw angle to target
    alpha = math.atan2((goal.start.y+goal.normal.y*offset)-constants.Y_SHOU,
                       (goal.start.x+goal.normal.x*offset)-constants.X_SHOU)
    h = math.sqrt(pow((goal.start.y+goal.normal.y*offset)-constants.Y_SHOU, 2) +
                  pow((goal.start.x+goal.normal.x*offset)-constants.X_SHOU, 2))
    l = constants.Y_SHOU - constants.HAND_Y_OFFSET
    beta = math.asin(l/h)
    # align scoop above target point
    sequence.plan_to_named_joint_positions(
      j_shou_yaw = alpha + beta,
      j_shou_pitch = math.pi / 2,
      j_prox_pitch = -math.pi / 2,
      j_dist_pitch = 0.0,
      j_hand_yaw = 0.0,
      j_scoop_yaw = 0.0
    )
    # once aligned to move goal and offset, place scoop tip at surface target offset
    sequence.plan_to_position(goal.start)
    # drive scoop along anti-normal vector by the search distance
    sequence.plan_to_translation(
        math3d.scalar_multiply(-goal.search_distance, goal.normal)
    )
    return sequence.merge()

  def execute_action(self, goal):
    GROUND_REFERENCE_FRAME = 'base_link'
    GROUND_POKER_LINK = 'l_scoop_tip'

    detector = GroundDetector(GROUND_REFERENCE_FRAME, GROUND_POKER_LINK)

    # define the callback locally to avoid making detector a member variable
    def ground_detect_cb():
      # publish feedback
      self._publish_feedback(current=self._arm_tip_monitor.get_link_position())
      # check if ground has been detected
      if detector.was_ground_detected():
        self._arm.stop_trajectory_silently()

    self._arm.move_group_scoop.set_planner_id('RRTstar')
    # Reset faults messages before the arm start moving
    self.fault_handler.reset_arm_faults()
    try:
      self._arm.checkout_arm(self.name)
      # TODO: split guarded_move trajectory into 2 parts so that ground
      #       detection can be started before the second execute_trajectory is
      #       called
      trajectory = self.plan_trajectory(goal)
      self._arm.execute_arm_trajectory(trajectory,
        action_feedback_cb=ground_detect_cb)
    except ArmError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err), final=Point())
      if isinstance(err, ArmPlanningError):
        self.fault_handler.set_arm_faults(ArmFaultsStatus.TRAJECTORY_GENERATION)
    else:
      self._arm.checkin_arm(self.name)
      if detector.was_ground_detected():
        ground_pos = detector.get_ground_position()
        self._pub_result.publish(True, GROUND_REFERENCE_FRAME, ground_pos)
        self._set_succeeded("Ground detected", final=ground_pos, success=True)
      else:
        self._pub_result.publish(False, '', Point())
        self._set_succeeded("No ground detected", final=Point(), success=False)
    finally:
      self._arm.move_group_scoop.set_planner_id('RRTstar')


class ArmUnstowServer(mixins.ArmTrajectoryMixin, ActionServerBase):

  name          = 'ArmUnstow'
  action_type   = owl_msgs.msg.ArmUnstowAction
  goal_type     = owl_msgs.msg.ArmUnstowGoal
  feedback_type = owl_msgs.msg.ArmUnstowFeedback
  result_type   = owl_msgs.msg.ArmUnstowResult

  fault_handler = faults.ArmFaultHandler()

  def plan_trajectory(self, _goal):
    sequence = TrajectorySequence(self._arm.robot, self._arm.move_group_scoop)
    sequence.plan_to_target('arm_unstowed')
    return sequence.merge()


class ArmStowServer(mixins.ArmTrajectoryMixin, ActionServerBase):

  name          = 'ArmStow'
  action_type   = owl_msgs.msg.ArmStowAction
  goal_type     = owl_msgs.msg.ArmStowGoal
  feedback_type = owl_msgs.msg.ArmStowFeedback
  result_type   = owl_msgs.msg.ArmStowResult

  fault_handler = faults.ArmFaultHandler()

  def plan_trajectory(self, _goal):
    sequence = TrajectorySequence(self._arm.robot, self._arm.move_group_scoop)
    sequence.plan_to_target('arm_stowed')
    return sequence.merge()


class TaskGrindServer(mixins.GrinderTrajectoryMixin, ActionServerBase):

  name          = 'TaskGrind'
  action_type   = owl_msgs.msg.TaskGrindAction
  goal_type     = owl_msgs.msg.TaskGrindGoal
  feedback_type = owl_msgs.msg.TaskGrindFeedback
  result_type   = owl_msgs.msg.TaskGrindResult

  fault_handler = faults.TaskFaultHandler()

  def publish_feedback_cb(self):
    self._publish_feedback(current=self._arm_tip_monitor.get_link_position())

  def plan_trajectory(self, goal):
    APPROACH_DISTANCE = 0.25 # meters
    # distance between the backward and forward linear paths
    SEGMENT_SEPARATION_DISTANCE = 0.08 # meters

    # NOTE: grind_point lies halfway between the two segments in the center of
    #       the trench
    grind_point = Point(goal.x_start, goal.y_start, goal.ground_position)
    yaw = _compute_workspace_shoulder_yaw(grind_point.x, grind_point.y)
    # define variables in perpendicular configuration (left-to-right of lander)
    trench_direction = Vector3(math.sin(yaw), -math.cos(yaw), 0.0)
    # NOTE: this grinder design can grind in any direction, so yaw can be zero
    grind_orientation = math3d.quaternion_from_euler(math.pi, math.pi / 2, 0)
    # compute the start of segment1 from grind_point
    segment1_offset_from_grind_point = math3d.add(
      Vector3(-SEGMENT_SEPARATION_DISTANCE / 2, 0, 0),
      math3d.scalar_multiply(-goal.length / 2, trench_direction)
    )
    segment_separation = math3d.scalar_multiply(
      -SEGMENT_SEPARATION_DISTANCE, math3d.orthogonal(trench_direction))
    if goal.parallel:
      # rotate segment defining vectors so they now run parallel
      rot_to_parallel = math3d.quaternion_from_euler(0, 0, math.pi / 2)
      segment1_offset_from_grind_point = math3d.quaternion_rotate(
        rot_to_parallel, segment1_offset_from_grind_point)
      segment_separation = math3d.quaternion_rotate(rot_to_parallel,
                                                    segment_separation)
      trench_direction = math3d.quaternion_rotate(rot_to_parallel,
                                                  trench_direction)
    # define entry position and positions for first segment of grind motion
    segment1_start_surface = math3d.add(grind_point, segment1_offset_from_grind_point)
    entry_approach = math3d.add(segment1_start_surface,
                                Vector3(0, 0, APPROACH_DISTANCE))
    segment1_start_bottom = math3d.subtract(segment1_start_surface,
                                            Vector3(0, 0, goal.depth))
    segment1_end_bottom = math3d.add(segment1_start_bottom,
      math3d.scalar_multiply(goal.length, trench_direction))
    # define positions for second segment of grind motion when it moves backward
    segment2_start_bottom = math3d.add(segment1_end_bottom,
                                       segment_separation)
    segment2_end_bottom = math3d.add(segment1_start_bottom, segment_separation)
    exit_retract = math3d.add(entry_approach, segment_separation)

    sequence = TrajectorySequence(
      self._arm.robot, self._arm.move_group_grinder, 'l_grinder_tip')
    sequence.plan_to_named_joint_positions(
      j_shou_yaw = yaw,
      j_shou_pitch = math.pi / 2,
      j_prox_pitch = -math.pi / 2,
      j_dist_pitch = 0.0,
      j_hand_yaw = -2 * math.pi / 3,
      j_grinder = 0.0
    )
    # place grinder directly above its terrain entry point
    sequence.plan_to_pose(Pose(entry_approach, grind_orientation))
    # enter terrain at the start of segment 1
    sequence.plan_to_position(segment1_start_bottom)
    # perform segment 1, moving away from grind_point
    sequence.plan_linear_path_to_pose(
      Pose(segment1_end_bottom, grind_orientation))
    # shift along segment separation direction to the start of segment 2
    sequence.plan_to_position(segment2_start_bottom)
    # perform segment 2, moving towards grind_point
    sequence.plan_linear_path_to_pose(
      Pose(segment2_end_bottom, grind_orientation))
    # retract out of terrain
    sequence.plan_to_position(exit_retract)
    return sequence.merge()

class TaskScoopCircularServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                              ActionServerBase):

  name          = 'TaskScoopCircular'
  action_type   = owl_msgs.msg.TaskScoopCircularAction
  goal_type     = owl_msgs.msg.TaskScoopCircularGoal
  feedback_type = owl_msgs.msg.TaskScoopCircularFeedback
  result_type   = owl_msgs.msg.TaskScoopCircularResult

  fault_handler = faults.TaskFaultHandler()

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def plan_trajectory(self, goal):
    # TODO:
    #  1. implement normal parameter
    #  2. implement scoop_angle parameter

    RADIUS = 0.4 # meters
    ARC = math.pi / 2
    RETRACT_DISTANCE = 0.2 # meters

    # NOTE: dig point is on the surface in the center of the circular trench
    dig_point = self.transform_to_planning_frame(
      self.get_intended_position(goal.frame, goal.relative, goal.point)).point
    # place end-effector above trench position
    yaw = _compute_workspace_shoulder_yaw(dig_point.x, dig_point.y)
    trench_bottom = Point(dig_point.x,
                          dig_point.y,
                          dig_point.z - goal.depth)
    # center of the circular arc
    center = math3d.add(trench_bottom, Vector3(0, 0, RADIUS))
    # rotates a downward facing point of contact (POC) on the circle to the
    # start and end of the perpendicular downward arc trajectory
    rot_down_to_start = math3d.quaternion_from_euler(ARC / 2, 0, yaw)
    rot_down_to_end = math3d.quaternion_from_euler(-ARC / 2, 0, yaw)
    # rotates from the scoops identity orientation (bottom up, facing away from
    # lander) to its perpendicular mid-scooping orientation (bottom down, facing
    # to the lander's right)
    rot_scoop_to_down = math3d.quaternion_from_euler(math.pi, 0, -math.pi / 2)
    if goal.parallel:
      # modify rotations so they describe the parallel downward arc trajectory
      rot_to_parallel = math3d.quaternion_from_euler(0, 0, math.pi / 2)
      rot_down_to_start = math3d.quaternion_multiply(rot_to_parallel,
                                                     rot_down_to_start)
      rot_down_to_end = math3d.quaternion_multiply(rot_to_parallel,
                                                   rot_down_to_end)
    # compute the start and end POCs along the circle by rotating the downward
    # facing POC into position using the rot_down_to_* quaternions
    poc1 = math3d.quaternion_rotate(rot_down_to_start, Vector3(0, 0, -RADIUS))
    poc2 = math3d.quaternion_rotate(rot_down_to_end, Vector3(0, 0, -RADIUS))
    # convert POCs back to BASE frame
    p1 = math3d.add(poc1, center)
    p2 = math3d.add(poc2, center)
    # acquire BASE frame orientations by combining the rotation required to
    # rotate the scoop into its mid-scooping orientation at the downward POC
    # with the rot_down_to_* rotations
    o1 = math3d.quaternion_multiply(rot_down_to_start, rot_scoop_to_down)
    o2 = math3d.quaternion_multiply(rot_down_to_end, rot_scoop_to_down)

    sequence = TrajectorySequence(
      self._arm.robot, self._arm.move_group_scoop, 'l_scoop_tip')
    sequence.plan_to_named_joint_positions(
      j_shou_yaw = yaw,
      j_shou_pitch = math.pi / 2,
      j_prox_pitch = -math.pi / 2,
      j_dist_pitch = 0.0,
      j_hand_yaw = 0.0,
      j_scoop_yaw = math.pi / 2 if goal.parallel else 0.0
    )
    # move arm to start of downward circular arc, with scoop facing down
    sequence.plan_to_pose(Pose(p1, o1))
    # move through downward circular arc and end with scoop pitched up
    sequence.plan_circular_path_to_pose(Pose(p2, o2), center)
    # retract out of the trench so the next arm movement can be made safely
    sequence.plan_to_z(dig_point.z + RETRACT_DISTANCE)
    return sequence.merge()


class TaskScoopLinearServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                            ActionServerBase):

  name          = 'TaskScoopLinear'
  action_type   = owl_msgs.msg.TaskScoopLinearAction
  goal_type     = owl_msgs.msg.TaskScoopLinearGoal
  feedback_type = owl_msgs.msg.TaskScoopLinearFeedback
  result_type   = owl_msgs.msg.TaskScoopLinearResult

  fault_handler = faults.TaskFaultHandler()

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def plan_trajectory(self, goal):
    # TODO:
    #  1. normal parameter

    APPROACH_DISTANCE = 0.22 # meters
    RETRACT_DISTANCE = 0.16 # meters
    ENTRY_RADIUS = 0.2 # meters
    EXIT_RADIUS = 0.4 # meters
    ENTRY_PITCH = math.pi / 2
    # this pitch was picked to avoid hand collision with terrain
    EXIT_PITCH = -0.8 # radians

    # NOTE: commanded dig point is halfway between the start of the entry
    #       circular trajectory and the start of the exit circular trajectory
    dig_point = self.transform_to_planning_frame(
      self.get_intended_position(goal.frame, goal.relative, goal.point)).point
    yaw = _compute_workspace_shoulder_yaw(dig_point.x, dig_point.y)
    trench_direction = Vector3(math.cos(yaw), math.sin(yaw), 0.0)
    # orientations scoop will transition between
    digging_orientation = math3d.quaternion_from_euler(math.pi, 0, yaw)
    entry_orietation = math3d.quaternion_from_euler(math.pi, ENTRY_PITCH, yaw)
    exit_orientation = math3d.quaternion_from_euler(math.pi, EXIT_PITCH, yaw)
    # point on the surface above where linear movement starts
    linear_start_surface = math3d.add(dig_point,
      math3d.scalar_multiply(-goal.length / 2, trench_direction))
    # point under the surface where linear movement starts
    linear_start_bottom = math3d.add(linear_start_surface,
                                     Vector3(0, 0, -goal.depth))
    # point under the surface where linear movement ends
    linear_end_bottom = math3d.add(linear_start_bottom,
      math3d.scalar_multiply(goal.length, trench_direction))
    # center of the circular entry arc
    entry_circle_center = math3d.add(linear_start_bottom,
                                     Vector3(0, 0, ENTRY_RADIUS))
    # center of the circular exit arc
    exit_circle_center = math3d.add(linear_end_bottom,
                                    Vector3(0, 0, EXIT_RADIUS))
    # rotate around center by entry arc
    entry_rot = math3d.quaternion_from_euler(0, ENTRY_PITCH, yaw)
    linear_start_poc = math3d.subtract(linear_start_bottom, entry_circle_center)
    entry_arc_start_poc = math3d.quaternion_rotate(entry_rot, linear_start_poc)
    entry_arc_start = math3d.add(entry_arc_start_poc, entry_circle_center)
    # place approach directly above the start of the entry arc
    entry_approach = copy(entry_arc_start)
    entry_approach.z = dig_point.z + APPROACH_DISTANCE
    # rotate around center by exit arc
    exit_rot = math3d.quaternion_from_euler(0, EXIT_PITCH, yaw)
    linear_end_poc = math3d.subtract(linear_end_bottom, exit_circle_center)
    exit_arc_end_poc = math3d.quaternion_rotate(exit_rot, linear_end_poc)
    exit_arc_end = math3d.add(exit_arc_end_poc, exit_circle_center)
    # z-position scoop will retract to after exit
    exit_retract_z = dig_point.z + RETRACT_DISTANCE

    sequence = TrajectorySequence(
      self._arm.robot, self._arm.move_group_scoop, 'l_scoop_tip')
    # place end-effector above trench position
    sequence.plan_to_named_joint_positions(
      j_shou_yaw = yaw,
      j_shou_pitch = math.pi / 2,
      j_prox_pitch = -math.pi / 2,
      j_dist_pitch = 0.0,
      j_hand_yaw = 0.0,
      j_scoop_yaw = math.pi / 2
    )
    # approach terrain while rotating into entry orientation
    sequence.plan_to_pose(Pose(entry_approach, entry_orietation))
    # place scoop tip at the start of the circular entry arc while maintaining
    # entry orientation
    sequence.plan_to_position(entry_arc_start)
    # rotate scoop tip into terrain
    sequence.plan_circular_path_to_pose(
      Pose(linear_start_bottom, digging_orientation),
      entry_circle_center
    )
    # move the scoop along a linear path to the end of the trench
    sequence.plan_linear_path_to_pose(
      Pose(linear_end_bottom, digging_orientation))
    # pitch scoop upward and out of the exit point
    sequence.plan_circular_path_to_pose(Pose(exit_arc_end, exit_orientation),
                                        exit_circle_center)
    # retract up from terrain while maintaining exit orientation
    # NOTE: only required for especially deep digs
    if sequence.get_final_pose().position.z < exit_retract_z:
      sequence.plan_to_z(exit_retract_z)
    return sequence.merge()


class TaskDiscardSampleServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                              ActionServerBase):

  name          = 'TaskDiscardSample'
  action_type   = owl_msgs.msg.TaskDiscardSampleAction
  goal_type     = owl_msgs.msg.TaskDiscardSampleGoal
  feedback_type = owl_msgs.msg.TaskDiscardSampleFeedback
  result_type   = owl_msgs.msg.TaskDiscardSampleResult
  goal_group_id = ow_lander.msg.ActionGoalStatus.TASK_GOAL

  fault_handler = faults.TaskFaultHandler()

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def plan_trajectory(self, goal):
    discard_surface_pos = self.transform_to_planning_frame(
      self.get_intended_position(goal.frame, goal.relative, goal.point)).point
    self._arm.move_group_scoop.set_planner_id("RRTstar")
    try:
      sequence = TrajectorySequence(
        self._arm.robot, self._arm.move_group_scoop, 'l_scoop')
      # move scoop to a pose above the discard point that holds the sample
      D2R = math.pi / 180
      held_euler = (
          -179 * D2R,
          -20  * D2R,
          -90  * D2R
      )
      held_pose = Pose(
        position = math3d.add(discard_surface_pos, Point(0, 0, goal.height)),
        orientation = math3d.quaternion_from_euler(*held_euler)
      )
      sequence.plan_to_pose(held_pose)

      # NOTE: This code does nothing since pos_constraint always remains a
      #       local variable. Saved in case the constraint is useful.
      # # adding position constraint on the solution so that the tip does not
      # # diverge to get to the solution.
      # pos_constraint = PositionConstraint()
      # pos_constraint.header.frame_id = "base_link"
      # pos_constraint.link_name = "l_scoop"
      # pos_constraint.target_point_offset.x = 0.1
      # pos_constraint.target_point_offset.y = 0.1
      # # rotate scoop to discard sample at current location begin
      # pos_constraint.target_point_offset.z = 0.1
      # pos_constraint.constraint_region.primitives.append(
      #     SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
      # pos_constraint.weight = 1

      # point front of scoop downward to discard sample
      dumped_euler = (
          180 * D2R,
          90  * D2R,
          -90 * D2R
      )
      dumped_quat = math3d.quaternion_from_euler(*dumped_euler)
      sequence.plan_to_orientation(dumped_quat)
      return sequence.merge()
    except ArmPlanningError as err:
      raise err
    finally:
      # TrajectorySequence calls may throw ArmPlanningError, which is handled
      # by ArmTrajectoryMixin, but they must be caught and passed on here so the
      # planner ID may be set back to RRTConnect before this method ends
      self._arm.move_group_scoop.set_planner_id("RRTConnect")


class TaskDeliverSampleServer(mixins.ArmTrajectoryMixin, ActionServerBase):

  name          = 'TaskDeliverSample'
  action_type   = owl_msgs.msg.TaskDeliverSampleAction
  goal_type     = owl_msgs.msg.TaskDeliverSampleGoal
  feedback_type = owl_msgs.msg.TaskDeliverSampleFeedback
  result_type   = owl_msgs.msg.TaskDeliverSampleResult

  fault_handler = faults.TaskFaultHandler()

  def plan_trajectory(self, _goal):
    self._arm.move_group_scoop.set_planner_id("RRTstar")
    try:
      sequence = TrajectorySequence(
        self._arm.robot, self._arm.move_group_scoop, 'l_scoop')
      sequence.plan_to_target("arm_deliver_staging_1")
      sequence.plan_to_target("arm_deliver_staging_2")
      sequence.plan_to_target("arm_deliver_final")
      return sequence.merge()
    except ArmPlanningError as err:
      raise err
    finally:
      # TrajectorySequence calls may throw ArmPlanningError, which is handled
      # by ArmTrajectoryMixin, but they must be caught and passed on here so the
      # planner ID may be set back to RRTConnect before this method ends
      self._arm.move_group_scoop.set_planner_id("RRTConnect")

class ArmMoveCartesianServer(mixins.FrameMixin, mixins.ArmActionMixin,
                             ActionServerBase):

  name          = 'ArmMoveCartesian'
  action_type   = owl_msgs.msg.ArmMoveCartesianAction
  goal_type     = owl_msgs.msg.ArmMoveCartesianGoal
  feedback_type = owl_msgs.msg.ArmMoveCartesianFeedback
  result_type   = owl_msgs.msg.ArmMoveCartesianResult

  fault_handler = faults.ArmFaultHandler()

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def publish_feedback_cb(self):
    self._publish_feedback(pose=self._arm_tip_monitor.get_link_pose())

  def execute_action(self, goal):
    # Reset faults messages before the arm start moving
    self.fault_handler.reset_arm_faults()
    try:
      intended_pose_stamped = self.get_intended_pose(goal.frame, goal.relative,
                                                     goal.pose)
    except ArmExecutionError as err:
      self._set_aborted(str(err))
      return
    try:
      self._arm.checkout_arm(self.name)
      comparison_transform = self.get_comparison_transform(
        intended_pose_stamped.header.frame_id)
      trajectory = self.plan_end_effector_to_pose(intended_pose_stamped)
      self._arm.execute_arm_trajectory(trajectory,
        action_feedback_cb=self.publish_feedback_cb)
    except ArmError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        final_pose=self._arm_tip_monitor.get_link_pose())
      if isinstance(err, ArmPlanningError):
        self.fault_handler.set_arm_faults(ArmFaultsStatus.TRAJECTORY_GENERATION)
      return
    else:
      self._arm.checkin_arm(self.name)
      results = {'final_pose': self._arm_tip_monitor.get_link_pose()}
      if not self.verify_pose_reached(intended_pose_stamped,
                                      comparison_transform):
        self._set_aborted("Failed to reach intended pose", **results)
        return
      self._set_succeeded(f"{self.name} trajectory succeeded", **results)


class ArmMoveCartesianGuardedServer(mixins.FrameMixin, mixins.ArmActionMixin,
                                    ActionServerBase):

  name          = 'ArmMoveCartesianGuarded'
  action_type   = owl_msgs.msg.ArmMoveCartesianGuardedAction
  goal_type     = owl_msgs.msg.ArmMoveCartesianGuardedGoal
  feedback_type = owl_msgs.msg.ArmMoveCartesianGuardedFeedback
  result_type   = owl_msgs.msg.ArmMoveCartesianGuardedResult

  fault_handler = faults.ArmFaultHandler()

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def execute_action(self, goal):
    # Reset faults messages before the arm start moving
    self.fault_handler.reset_arm_faults()
    try:
      intended_pose_stamped = self.get_intended_pose(goal.frame, goal.relative,
                                                     goal.pose)
    except ArmExecutionError as err:
      self._set_aborted(str(err))
      return
    # monitor F/T sensor and define a callback to check its status
    monitor = FTSensorThresholdMonitor(force_threshold=goal.force_threshold,
                                       torque_threshold=goal.torque_threshold)
    def guarded_cb():
      self._publish_feedback(
        pose=self._arm_tip_monitor.get_link_pose(),
        force=monitor.get_force(),
        torque=monitor.get_torque()
      )
      if monitor.threshold_breached():
        self._arm.stop_trajectory_silently()
    # perform action
    try:
      self._arm.checkout_arm(self.name)
      plan = self.plan_end_effector_to_pose(intended_pose_stamped)
      comparison_transform = self.get_comparison_transform(
        intended_pose_stamped.header.frame_id)
      self._arm.execute_arm_trajectory(plan, action_feedback_cb=guarded_cb)
    except ArmError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        final_pose=self._arm_tip_monitor.get_link_pose(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque())
      if isinstance(err, ArmPlanningError):
        self.fault_handler.set_arm_faults(ArmFaultsStatus.TRAJECTORY_GENERATION)
      return
    else:
      self._arm.checkin_arm(self.name)
      results = {
        'final_pose': self._arm_tip_monitor.get_link_pose(),
        'final_force': monitor.get_force(),
        'final_torque': monitor.get_torque()
      }
      if not monitor.threshold_breached() and \
          not self.verify_pose_reached(intended_pose_stamped,
                                       comparison_transform):
        # pose was not reached due to planning/monitor error
        self._set_aborted(NO_THRESHOLD_BREACH_MESSAGE, **results)
        return
      self._set_succeeded(
        _format_guarded_move_success_message(self.name, monitor),
        **results
      )


class ArmFindSurfaceServer(mixins.FrameMixin, mixins.ArmActionMixin,
                           ActionServerBase):

  name          = 'ArmFindSurface'
  action_type   = owl_msgs.msg.ArmFindSurfaceAction
  goal_type     = owl_msgs.msg.ArmFindSurfaceGoal
  feedback_type = owl_msgs.msg.ArmFindSurfaceFeedback
  result_type   = owl_msgs.msg.ArmFindSurfaceResult

  fault_handler = faults.ArmFaultHandler()

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def publish_feedback_cb(self, distance=0, force=0, torque=0):
    self._publish_feedback(
      pose=self.get_end_effector_pose(constants.FRAME_ID_BASE).pose,
      distance=distance,
      force=force,
      torque=torque
    )

  def execute_action(self, goal):
    # Reset faults messages before the arm start moving
    self.fault_handler.reset_arm_faults()
    # the normal vector direction the scoop's bottom faces in its frame
    SCOOP_DOWNWARD = Vector3(0, 0, 1)
    try:
      # transform only position
      estimated_surface_stamped = self.transform_to_planning_frame(
        self.get_intended_position(goal.frame, goal.relative, goal.position)
      )
      # normal is always considered in base_link regardless of frame parameter
      normal = self.validate_normalization(goal.normal)
    except ArmExecutionError as err:
      self._set_aborted(str(err))
      return
    # orient scoop so that the bottom points opposite to the normal
    orientation = math3d.quaternion_rotation_between(SCOOP_DOWNWARD, normal)
    # pose before end-effector is driven towards surface
    intended_start_pose_stamped = PoseStamped(
      header=estimated_surface_stamped.header,
      pose=Pose(
        # begin a distance along the anti-normal direction from surface position
        position=math3d.add(
          estimated_surface_stamped.point,
          math3d.scalar_multiply(-goal.distance, normal)
        ),
        orientation=orientation
      )
    )
    # pose after end-effector has driven its maximum distance towards surface
    # if there is no surface, the end-effector will reach this pose
    intended_end_pose_stamped = PoseStamped(
      header=estimated_surface_stamped.header,
      pose=Pose(
        # end an overdrive along the normal direction from the surface position
        position=math3d.add(
          estimated_surface_stamped.point,
          math3d.scalar_multiply(goal.overdrive, normal)
        ),
        orientation=orientation
      )
    )
    # move to setup pose prior to surface approach
    try:
      self._arm.checkout_arm(self.name)
      trajectory_setup = self.plan_end_effector_to_pose(
        intended_start_pose_stamped)
      comparison_transform = self.get_comparison_transform(
        intended_start_pose_stamped.header.frame_id)
      self._arm.execute_arm_trajectory(trajectory_setup,
        action_feedback_cb=self.publish_feedback_cb)
    except ArmError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err) + " - Setup trajectory failed",
        final_pose=self.get_end_effector_pose(constants.FRAME_ID_BASE).pose,
        final_distance=0, final_force=0, final_torque=0)
      if isinstance(err, ArmPlanningError):
        self.fault_handler.set_arm_faults(ArmFaultsStatus.TRAJECTORY_GENERATION)
      return
    else:
      self._arm.checkin_arm(self.name)
      if not self.verify_pose_reached(intended_start_pose_stamped,
                                      comparison_transform):
       self._set_aborted("Failed to reach setup pose.",
          final_pose=self.get_end_effector_pose(constants.FRAME_ID_BASE).pose,
          final_distance=0, final_force=0, final_torque=0)
       return
    # local function to compute progress of the action during surface approach
    def compute_distance():
      pose = self.get_end_effector_pose(constants.FRAME_ID_BASE).pose
      d = math3d.subtract(pose.position, estimated_surface_stamped.point)
      return math3d.norm(d)
    # setup F/T monitor and its callback
    monitor = FTSensorThresholdMonitor(force_threshold=goal.force_threshold,
                                       torque_threshold=goal.torque_threshold)
    def guarded_cb():
      self.publish_feedback_cb(
        compute_distance(), monitor.get_force(), monitor.get_torque())
      if monitor.threshold_breached():
        self._arm.stop_trajectory_silently()
    # move towards surface until F/T is breached or overdrive distance reached
    try:
      self._arm.checkout_arm(self.name)
      trajectory_approach = self.plan_end_effector_to_pose(
        intended_end_pose_stamped)
      comparison_transform = self.get_comparison_transform(
        intended_start_pose_stamped.header.frame_id)
      self._arm.execute_arm_trajectory(trajectory_approach,
        action_feedback_cb=guarded_cb)
    except ArmError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err) + " - Surface approach trajectory failed",
        final_pose=self.get_end_effector_pose(constants.FRAME_ID_BASE).pose,
        final_distance=compute_distance(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque()
      )
      if isinstance(err, ArmPlanningError):
        self.fault_handler.set_arm_faults(ArmFaultsStatus.TRAJECTORY_GENERATION)
    else:
      self._arm.checkin_arm(self.name)
      results = {
        'final_pose' : self.get_end_effector_pose(constants.FRAME_ID_BASE).pose,
        'final_distance' : compute_distance(),
        'final_force'    : monitor.get_force(),
        'final_torque'   : monitor.get_torque(),
      }
      if not monitor.threshold_breached() and \
          not self.verify_pose_reached(intended_end_pose_stamped,
                                       comparison_transform):
        # pose was not reached due to planning/monitor error
        self._set_aborted(NO_THRESHOLD_BREACH_MESSAGE, **results)
      elif not monitor.threshold_breached():
        self._set_succeeded("No surface was found", **results)
      else:
        msg = _format_guarded_move_success_message(self.name, monitor)
        surface_position = results['final_pose'].position
        msg += f". Surface found at ({surface_position.x:0.3f}, "
        msg +=                     f"{surface_position.y:0.3f}, "
        msg +=                     f"{surface_position.z:0.3f})"
        self._set_succeeded(msg, **results)


class ArmMoveJointServer(mixins.ModifyJointValuesMixin, ActionServerBase):

  name          = 'ArmMoveJoint'
  action_type   = owl_msgs.msg.ArmMoveJointAction
  goal_type     = owl_msgs.msg.ArmMoveJointGoal
  feedback_type = owl_msgs.msg.ArmMoveJointFeedback
  result_type   = owl_msgs.msg.ArmMoveJointResult

  fault_handler = faults.ArmFaultHandler()

  def modify_joint_positions(self, goal):
    pos = self._arm_joints_monitor.get_joint_positions()
    if goal.joint < 0 or goal.joint >= len(pos):
      raise ArmExecutionError("Provided joint index is not within range")
    if goal.relative:
      pos[goal.joint] += goal.angle
    else:
      pos[goal.joint] = goal.angle
    return pos


class ArmMoveJointsServer(mixins.ModifyJointValuesMixin, ActionServerBase):

  name          = 'ArmMoveJoints'
  action_type   = owl_msgs.msg.ArmMoveJointsAction
  goal_type     = owl_msgs.msg.ArmMoveJointsGoal
  feedback_type = owl_msgs.msg.ArmMoveJointsFeedback
  result_type   = owl_msgs.msg.ArmMoveJointsResult

  fault_handler = faults.ArmFaultHandler()

  def modify_joint_positions(self, goal):
    pos = self._arm_joints_monitor.get_joint_positions()
    if len(goal.angles) != len(pos):
      raise ArmExecutionError("Number of angle positions provided does not " \
                              "much the number of joints in the arm move group")
    for i in range(len(pos)):
      if goal.relative:
        pos[i] += goal.angles[i]
      else:
        pos[i] = goal.angles[i]
    return pos


# inherit from ArmMoveJoints since lots of code can be reused
class ArmMoveJointsGuardedServer(ArmMoveJointsServer):

  name          = 'ArmMoveJointsGuarded'
  action_type   = owl_msgs.msg.ArmMoveJointsGuardedAction
  goal_type     = owl_msgs.msg.ArmMoveJointsGuardedGoal
  feedback_type = owl_msgs.msg.ArmMoveJointsGuardedFeedback
  result_type   = owl_msgs.msg.ArmMoveJointsGuardedResult

  fault_handler = faults.ArmFaultHandler()

  # redefine execute_action to enable FT monitor
  def execute_action(self, goal):
    # Reset faults messages before the arm start moving
    self.fault_handler.reset_arm_faults()
    monitor = FTSensorThresholdMonitor(force_threshold=goal.force_threshold,
                                       torque_threshold=goal.torque_threshold)
    def guarded_cb():
      self._publish_feedback(
        angles=self._arm_joints_monitor.get_joint_positions(),
        force=monitor.get_force(),
        torque=monitor.get_torque()
      )
      if monitor.threshold_breached():
        self._arm.stop_trajectory_silently()
    try:
      self._arm.checkout_arm(self.name)
      new_positions = self.modify_joint_positions(goal)
      sequence = TrajectorySequence(self._arm.robot, self._arm.move_group_scoop)
      sequence.plan_to_joint_positions(new_positions)
      self._arm.execute_arm_trajectory(sequence.merge(),
        action_feedback_cb=guarded_cb)
    except ArmExecutionError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        final_angles=self._arm_joints_monitor.get_joint_positions(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque())
    except ArmPlanningError as err:
      self._arm.checkin_arm(self.name)
      self.fault_handler.set_arm_faults(ArmFaultsStatus.TRAJECTORY_GENERATION)
      self._set_aborted(str(err),
        final_angles=self._arm_joints_monitor.get_joint_positions(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque())
    else:
      self._arm.checkin_arm(self.name)
      if not monitor.threshold_breached() \
          and not self.angles_reached(new_positions):
        # angles were not reached due to planning/monitor error
        self._set_aborted(
          NO_THRESHOLD_BREACH_MESSAGE,
          final_angles=self._arm_joints_monitor.get_joint_positions(),
          final_force=monitor.get_force(),
          final_torque=monitor.get_torque()
        )
        return
      self._set_succeeded(
        _format_guarded_move_success_message(self.name, monitor),
        final_angles=self._arm_joints_monitor.get_joint_positions(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque()
      )


#############################
## NON-ARM RELATED ACTIONS
#############################

class LightSetIntensityServer(ActionServerBase):

  name          = 'LightSetIntensity'
  action_type   = owl_msgs.msg.LightSetIntensityAction
  goal_type     = owl_msgs.msg.LightSetIntensityGoal
  feedback_type = owl_msgs.msg.LightSetIntensityFeedback
  result_type   = owl_msgs.msg.LightSetIntensityResult

  def __init__(self):
    super(LightSetIntensityServer, self).__init__()
    self._start_server()

  def execute_action(self, goal):
    intensity = goal.intensity
    name = goal.name.lower()
    # check intensity range
    if intensity < 0.0 or intensity > 1.0:
      self._set_aborted(f"Intensity = {intensity} is out of range.")
      return
    # check for correct names
    if name == 'left':
      rospy.set_param('/OWLightControlPlugin/left_light', intensity)
    elif name == 'right':
      rospy.set_param('/OWLightControlPlugin/right_light', intensity)
    else:
      self._set_aborted(f"\'{name}\' is not a light identifier.")
      return
    self._set_succeeded(f"{name} light intensity set successfully.")

# the maximum time the camera actions will wait for the camera plugin to have
# subscribed to their control topics
SUBSCRIBER_TIMEOUT = 30

class CameraCaptureServer(ActionServerBase):

  name          = 'CameraCapture'
  action_type   = owl_msgs.msg.CameraCaptureAction
  goal_type     = owl_msgs.msg.CameraCaptureGoal
  feedback_type = owl_msgs.msg.CameraCaptureFeedback
  result_type   = owl_msgs.msg.CameraCaptureResult

  fault_handler = faults.CameraFaultHandler()

  def __init__(self):
    super(CameraCaptureServer, self).__init__()
    # set up interface for capturing a photograph with the camera
    self._pub_trigger = rospy.Publisher('/StereoCamera/left/image_trigger',
                                        Empty,
                                        queue_size=10)
    self._sub_point_cloud = rospy.Subscriber('/StereoCamera/points2',
                                             PointCloud2,
                                             self._handle_point_cloud)
    self.point_cloud_created = False
    if not wait_for_subscribers(self._pub_trigger, SUBSCRIBER_TIMEOUT):
      rospy.logwarn(f"No subscribers to topic {self._pub_trigger.name} after" \
                    f"waiting {SUBSCRIBER_TIMEOUT} seconds. CameraCapture " \
                    "may not work correctly as a result.")
    self._start_server()

  def _handle_point_cloud(self, points):
    """
    :type points: sensor_msgs.msg.PointCloud2
    """
    # CameraCapture was successful if a point cloud is received. However, there
    # does not appear to be a way to associate this point cloud with the
    # original trigger message. A trigger could have been sent without using
    # the CameraCapture action client.
    self.point_cloud_created = True

  def execute_action(self, goal):
    self.point_cloud_created = False

    self._pub_trigger.publish()

    # apply power usage for 2 cameras
    ### FIXME: Exposure does not appear to be simulated in time (e.g. a 10
    ###   second exposure will not take 10 seconds to complete), so this model
    ###   per camera power consumption does not actually work the way it should.
    per_camera_power_usage = rospy.get_param(
      '/ow_power_system/power_active_camera')
    PowerInterface().set_power_load('camera', per_camera_power_usage)

    # await point cloud or action preempt
    FREQUENCY = 10 # Hz
    TIMEOUT = 5   # seconds
    rate = rospy.Rate(FREQUENCY)
    for i in range(0, int(TIMEOUT * FREQUENCY)):
      # TODO: investigate what preempt's function is here and in other actions
      if self._is_preempt_requested():
        self._set_preempted("Action was preempted")
        PowerInterface().reset_power_load('camera')
        return
      if self.point_cloud_created:
        self._set_succeeded("Point cloud received")
        PowerInterface().reset_power_load('camera')
        return
      rate.sleep()
    self._set_aborted("Timed out waiting for point cloud")
    PowerInterface().reset_power_load('camera')

class CameraSetExposureServer(ActionServerBase):

  name          = 'CameraSetExposure'
  action_type   = owl_msgs.msg.CameraSetExposureAction
  goal_type     = owl_msgs.msg.CameraSetExposureGoal
  feedback_type = owl_msgs.msg.CameraSetExposureFeedback
  result_type   = owl_msgs.msg.CameraSetExposureResult
  goal_group_id = ow_lander.msg.ActionGoalStatus.CAMERA_GOAL

  fault_handler = faults.CameraFaultHandler()

  def __init__(self):
    super(CameraSetExposureServer, self).__init__()
    # set up interface for setting camera exposure
    self._pub_exposure = rospy.Publisher('/gazebo/plugins/camera_sim/exposure',
                                         Float64,
                                         queue_size=10)
    if not wait_for_subscribers(self._pub_exposure, SUBSCRIBER_TIMEOUT):
      rospy.logwarn(f"No subscribers to topic {self._pub_exposure.name} after" \
                    f"waiting {SUBSCRIBER_TIMEOUT} seconds. CameraSetExposure" \
                    " may not work correctly as a result.")
    self._start_server()

  def execute_action(self, goal):
    if goal.automatic:
      self._set_aborted(f'{self.name} action does not support autoexposure.')
      return

    # Set exposure if it is specified
    if goal.exposure > 0:
      self._pub_exposure.publish(goal.exposure)
      # There is no guarantee that exposure is set before the image is triggered
      # Pause to make it highly likely that exposure is received first.
      rospy.sleep(1)
      self._set_succeeded(f'Exposure set to {goal.exposure}')
    else:
      self._set_succeeded('Exposure not changed')


class DockIngestSampleServer(ActionServerBase):

  name          = 'DockIngestSample'
  action_type   = ow_lander.msg.DockIngestSampleAction
  goal_type     = ow_lander.msg.DockIngestSampleGoal
  feedback_type = ow_lander.msg.DockIngestSampleFeedback
  result_type   = ow_lander.msg.DockIngestSampleResult

  def __init__(self):
    super(DockIngestSampleServer, self).__init__()
    self._link_states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates,
                                             self._on_link_states_msg)
    self._link_states = None
    self._regolith_removed = False
    self._start_server()

  def _on_link_states_msg(self, msg):
    self._link_states = msg

  def _get_dock_pose(self):
    try:
      i = self._link_states.name.index('lander::lander_sample_dock_link')
    except ValueError:
      raise ActionError(
        "lander::lander_sample_dock_link not found in /gazebo/link_states")
    return self._link_states.pose[i]

  def _is_position_in_sample_dock(self, position, dock_pose):
    # transform world frame position to a sample dock frame position
    # FIXME: This must be done manual due to the tf inaccuracy caused by OW-1194
    transformed = math3d.subtract(position, dock_pose.position)
    transformed = math3d.quaternion_rotate(
      math3d.quaternion_inverse(dock_pose.orientation),
      transformed
    )
    # perform an axis-aligned box containment check since, in the sample dock's
    # frame, it happens to be an axis-aligned box
    X_DIM = 0.3 # meters
    Y_DIM = 0.05
    Z_DIM = 0.095
    # NOTE: This value can be found in lander_sample_dock.xacro, but not
    #   trivially. The y-value comes from the lander_sample_dock macro
    #   definition and is the y-value passed to the lander_sample_dock_link
    #   macro. The z-value comes from the fact that both the collision and
    #   visual meshes in the lander_sample_dock_link macro are defined with an
    #   origin offset by 0.025 in the +z direction. Combine these two
    #   adjustments together to get the value of OFFSET_RELATIVE_TO_FRAME.
    OFFSET_RELATIVE_TO_FRAME = Point(0.0, -0.33, 0.025)
    p = math3d.subtract(transformed, OFFSET_RELATIVE_TO_FRAME)
    if (    -X_DIM/2 < p.x < X_DIM/2
        and -Y_DIM/2 < p.y < Y_DIM/2
        and -Z_DIM/2 < p.z < Z_DIM/2 ):
      return True
    else:
      return False

  def _identify_active_regolith_in_sample_dock(self):
    regolith = list()
    dock_pose = self._get_dock_pose()
    for i in range(len(self._link_states.name)):
      name = self._link_states.name[i]
      position = self._link_states.pose[i].position
      if ("regolith_" in name
          and
          self._is_position_in_sample_dock(position, dock_pose)):
        regolith.append(name[:-6]) # strip "::link" from end of name
    return regolith

  def _remove_regolith_in_dock(self):
    regolith_to_remove = self._identify_active_regolith_in_sample_dock()
    if not regolith_to_remove:
      return False
    REMOVE_REGOLITH_SERVICE = '/ow_regolith/remove_regolith'
    rospy.wait_for_service(REMOVE_REGOLITH_SERVICE, timeout=10)
    service = rospy.ServiceProxy(REMOVE_REGOLITH_SERVICE, RemoveRegolith)
    _ = service(regolith_to_remove, True)
    return True

  def execute_action(self, _goal):
    if not wait_for_message(self._link_states, 10):
      self._set_aborted(
        "Timed out waiting for a message on /gazebo/link_states.",
        sample_ingested = False
      )
      return
    self._regolith_removed = False
    try:
      FREQUENCY = 1 #Hz
      TIMEOUT = 3 # seconds
      rate = rospy.Rate(FREQUENCY)
      iterations_since_last_removal = 0
      while iterations_since_last_removal < int(TIMEOUT * FREQUENCY):
        iterations_since_last_removal += 1
        if self._remove_regolith_in_dock():
          # reset timer
          iterations_since_last_removal = 0
          self._regolith_removed = True
        rate.sleep()
    except (rospy.ServiceException, rospy.ROSException) as err:
      rospy.logwarn(f"Service call failed: {err}")
      self._set_aborted(
        "Removal service failed",
        sample_ingested = self._regolith_removed
      )
      return
    except ActionError as err:
      self._set_aborted(
        f"Failed to verify which regolith are present in sample dock: {err}",
        sample_ingested = self._regolith_removed
      )
      return

    if self._regolith_removed:
      self._set_succeeded("Sample ingested", sample_ingested=True)
    else:
      self._set_succeeded("No sample was present in dock",
                          sample_ingested=False)


class PanTiltMoveJointsServer(mixins.PanTiltMoveMixin, ActionServerBase):

  name          = 'PanTiltMoveJoints'
  action_type   = owl_msgs.msg.PanTiltMoveJointsAction
  goal_type     = owl_msgs.msg.PanTiltMoveJointsGoal
  feedback_type = owl_msgs.msg.PanTiltMoveJointsFeedback
  result_type   = owl_msgs.msg.PanTiltMoveJointsResult

  fault_handler = faults.PanTiltFaultHandler()

  def execute_action(self, goal):
    try:
      not_preempted = self.move(pan = goal.pan, tilt = goal.tilt)
    except AntennaError as err:
      pan, tilt = self._ant_joints_monitor.get_joint_positions()
      self._set_aborted(str(err), pan_position=pan, tilt_position=tilt)
    else:
      pan, tilt = self._ant_joints_monitor.get_joint_positions()
      if not_preempted:
        self._set_succeeded("Reached commanded pan/tilt values",
          pan_position=pan, tilt_position=tilt)
      else:
        self._set_preempted("Action was preempted",
          pan_position=pan, tilt_position=tilt)

  def publish_feedback_cb(self):
    current_pan, current_tilt = self._ant_joints_monitor.get_joint_positions()
    self._publish_feedback(pan_position = current_pan,
                           tilt_position = current_tilt)


class PanServer(mixins.PanTiltMoveMixin, ActionServerBase):

  name          = 'Pan'
  action_type   = ow_lander.msg.PanAction
  goal_type     = ow_lander.msg.PanGoal
  feedback_type = ow_lander.msg.PanFeedback
  result_type   = ow_lander.msg.PanResult

  fault_handler = faults.PanTiltFaultHandler()

  def execute_action(self, goal):
    try:
      not_preempted = self.move(pan = goal.pan)
    except AntennaError as err:
      pan, _ = self._ant_joints_monitor.get_joint_positions()
      self._set_aborted(str(err), pan_position=pan)
    else:
      pan, _ = self._ant_joints_monitor.get_joint_positions()
      if not_preempted:
        self._set_succeeded("Reached commanded pan value", pan_position=pan)
      else:
        self._set_preempted("Action was preempted", pan_position=pan)

  def publish_feedback_cb(self):
    current_pan, _ = self._ant_joints_monitor.get_joint_positions()
    self._publish_feedback(pan_position = current_pan)


class TiltServer(mixins.PanTiltMoveMixin, ActionServerBase):

  name          = 'Tilt'
  action_type   = ow_lander.msg.TiltAction
  goal_type     = ow_lander.msg.TiltGoal
  feedback_type = ow_lander.msg.TiltFeedback
  result_type   = ow_lander.msg.TiltResult

  fault_handler = faults.PanTiltFaultHandler()

  def execute_action(self, goal):
    try:
      not_preempted = self.move(tilt = goal.tilt)
    except AntennaError as err:
      _, tilt = self._ant_joints_monitor.get_joint_positions()
      self._set_aborted(str(err), tilt_position=tilt)
    else:
      _, tilt = self._ant_joints_monitor.get_joint_positions()
      if not_preempted:
        self._set_succeeded("Reached commanded tilt value", tilt_position=tilt)
      else:
        self._set_preempted("Action was preempted", tilt_position=tilt)

  def publish_feedback_cb(self):
    _, current_tilt = self._ant_joints_monitor.get_joint_positions()
    self._publish_feedback(tilt_position = current_tilt)


class PanTiltMoveCartesianServer(mixins.PanTiltMoveMixin, ActionServerBase):

  name          = 'PanTiltMoveCartesian'
  action_type   = owl_msgs.msg.PanTiltMoveCartesianAction
  goal_type     = owl_msgs.msg.PanTiltMoveCartesianGoal
  feedback_type = owl_msgs.msg.PanTiltMoveCartesianFeedback
  result_type   = owl_msgs.msg.PanTiltMoveCartesianResult

  fault_handler = faults.PanTiltFaultHandler()

  def execute_action(self, goal):
    LOOKAT_FRAME = constants.FRAME_ID_BASE
    cam_center = FrameTransformer().lookup_transform(LOOKAT_FRAME,
                                                     'StereoCameraCenter_link')
    tilt_joint = FrameTransformer().lookup_transform(LOOKAT_FRAME,
                                                     'l_ant_panel')
    try:
      frame_id = mixins.FrameMixin.get_frame_id_from_index(goal.frame)
    except ActionError as err:
      self._set_aborted(str(err))
      return
    lookat = goal.point
    if frame_id != LOOKAT_FRAME:
      lookat = FrameTransformer().transform_geometry(
        goal.point, LOOKAT_FRAME, frame_id)
    if cam_center is None or tilt_joint is None or lookat is None:
      self._set_aborted("Failed to perform necessary transforms to compute "
                        "appropriate pan and tilt values.")
      return

    # The following computations make the approximation that the camera center
    # link lies directly above the tilt axis when tilt = 0. The error caused by
    # this assumption is small enough to be ignored.

    # compute the vector from the tilt joint to the lookat position
    tilt_to_lookat = math3d.subtract(lookat, tilt_joint.transform.translation)
    # pan is the +Z Euler angle of tilt_to_lookat
    # pi/2 must be added because pan's zero position faces in the -y direction
    pan_raw = math.atan2(tilt_to_lookat.y, tilt_to_lookat.x) + (math.pi / 2)
    pan = normalize_radians(pan_raw)
    # compute length of the lever arm between tilt joint and camera center
    l = math3d.norm(math3d.subtract(cam_center.transform.translation,
                                    tilt_joint.transform.translation))
    # Imagine the cameras are already pointed at the lookat position and that a
    # vector extends out from their midpoint to the lookat position. If we
    # approximate the angle between that vector and the camera lever arm to be
    # pi/2 then the vector, tilt_to_lookat, and the camera lever arm form a
    # right triangle. Therefore, the angle between tilt_to_lookat and the camera
    # lever arm can be approximated from only their lengths.
    a = math.acos(l / math3d.norm(tilt_to_lookat))
    # compute the angle between tilt_to_lookat and the X-Y plane
    b = math.atan2(tilt_to_lookat.z,
                   math.sqrt(tilt_to_lookat.x**2 + tilt_to_lookat.y**2))
    # The sum of a and b gives the angle between the desired camera lever arm
    # vector and the x-y plane.
    # Antenna tilt is measured from the +z axis, so a pi/2 is subtracted.
    # Finally, tilt rotates in reverse of the unit circle, so we multiply the
    # the result by -1.
    tilt_raw = -(a + b - (math.pi / 2))
    tilt = normalize_radians(tilt_raw)
    try:
      not_preempted = self.move(pan = pan, tilt = tilt)
    except AntennaError as err:
      self._set_aborted(str(err))
    else:
      if not_preempted:
        self._set_succeeded("Reached commanded pan/tilt values")
      else:
        self._set_preempted("Action was preempted")


class ActivateCommsServer(ActionServerBase):

  name          = 'ActivateComms'
  action_type   = ow_lander.msg.ActivateCommsAction
  goal_type     = ow_lander.msg.ActivateCommsGoal
  feedback_type = ow_lander.msg.ActivateCommsFeedback
  result_type   = ow_lander.msg.ActivateCommsResult

  def __init__(self):
    super(ActivateCommsServer, self).__init__()
    self._start_server()

  def execute_action(self, goal):

    # This action only draws a some amount of power from the battery for some
    # period of time. There is no simulation of mission to Earth communications,
    # and, for the purpose of this action, uplinking and downlinking are treated
    # the same and draw the same amount of power.

    power_load_while_active = rospy.get_param(
      '/ow_power_system/power_active_comms')

    PowerInterface().set_power_load('comms', power_load_while_active)
    # wait in simulation time
    rospy.sleep(goal.duration)
    PowerInterface().reset_power_load('comms')

    self._set_succeeded("Communication uplink/downlink succeeded.")
