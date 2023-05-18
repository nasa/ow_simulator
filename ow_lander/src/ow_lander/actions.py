# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines all lander actions"""

import math

import rospy
import owl_msgs.msg
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import PointCloud2
from ow_regolith.srv import RemoveRegolith
from ow_regolith.msg import Contacts
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Quaternion
from urdf_parser_py.urdf import URDF

import ow_lander.msg
from ow_lander import mixins
from ow_lander import math3d
from ow_lander import constants
from ow_lander.server import ActionServerBase
from ow_lander.common import normalize_radians
from ow_lander.exception import (ArmPlanningError, ArmExecutionError,
                                 AntennaPlanningError, AntennaExecutionError)
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

class ArmStopServer(mixins.ArmActionMixin, ActionServerBase):

  name          = 'ArmStop'
  action_type   = owl_msgs.msg.ArmStopAction
  goal_type     = owl_msgs.msg.ArmStopGoal
  feedback_type = owl_msgs.msg.ArmStopFeedback
  result_type   = owl_msgs.msg.ArmStopResult
  goal_group_id = None

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

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    # TODO: would it make sense to latch this?
    # TODO: is this publisher necessary?
    self._pub_result = rospy.Publisher('/guarded_move_result',
                                       ow_lander.msg.GuardedMoveFinalResult,
                                       queue_size=1)

  def plan_trajectory(self, goal):
    sequence = TrajectorySequence(
      'l_scoop', self._arm.robot, self._arm.move_group_scoop)
    # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
    targ_elevation = -0.2
    if (goal.point.z+targ_elevation) == 0:
      offset = goal.search_distance
    else:
      offset = (goal.point.z*goal.search_distance)/(goal.point.z+targ_elevation)
    # Compute shoulder yaw angle to target
    alpha = math.atan2((goal.point.y+goal.normal.y*offset)-constants.Y_SHOU,
                       (goal.point.x+goal.normal.x*offset)-constants.X_SHOU)
    h = math.sqrt(pow((goal.point.y+goal.normal.y*offset)-constants.Y_SHOU, 2) +
                  pow((goal.point.x+goal.normal.x*offset)-constants.X_SHOU, 2))
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
    sequence.plan_to_position(goal.point)
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
      self.publish_feedback_cb()
      # check if ground has been detected
      if detector.was_ground_detected():
        self._arm.stop_trajectory_silently()

    self._arm.move_group_scoop.set_planner_id('RRTstar')
    try:
      self._arm.checkout_arm(self.name)
      # TODO: split guarded_move trajectory into 2 parts so that ground
      #       detection can be started before the second execute_trajectory is
      #       called
      trajectory = self.plan_trajectory(goal)
      self._arm.execute_arm_trajectory(trajectory,
        action_feedback_cb=ground_detect_cb)
    except (ArmExecutionError, ArmPlanningError) as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err), final=Point())
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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.ARM_GOAL


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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.ARM_GOAL

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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.TASK_GOAL

  def publish_feedback_cb(self):
    self._publish_feedback(current=self._arm_tip_monitor.get_link_position())

  def plan_trajectory(self, goal):
    PREGRIND_HEIGHT = 0.25

    point = Point(goal.x_start, goal.y_start, goal.ground_position)
    depth = goal.depth
    length = goal.length
    parallel = goal.parallel

    yaw = _compute_workspace_shoulder_yaw(point.x, point.y)
    if parallel:
      R = math.sqrt(point.x*point.x+point.y*point.y)
      # adjust trench to fit scoop circular motion
      dx = 0.04*R*math.sin(yaw)  # Center dig_circular in grind trench
      dy = 0.04*R*math.cos(yaw)
      # Move starting point back to avoid scoop-terrain collision
      point.x = 0.9*(point.x + dx)
      point.y = 0.9*(point.y - dy)
    else:
      dx = 5*length/8*math.sin(yaw)
      dy = 5*length/8*math.cos(yaw)
      # Move starting point back to avoid scoop-terrain collision
      point.x = 0.97*(point.x - dx)
      point.y = 0.97*(point.y + dy)

    sequence = TrajectorySequence(
      self._arm.robot, self._arm.move_group_grinder, 'l_grinder')
    # place grinder above the start point
    pregrind_position = math3d.add(point, Vector3(0, 0, PREGRIND_HEIGHT))
    pregrind_pose = Pose(
      position = pregrind_position,
      orientation = Quaternion(0.70616885803, 0.0303977418722,
                               -0.706723318474, 0.0307192507001)
    )
    sequence.plan_to_pose(pregrind_pose)
    # enter terrain
    trench_bottom = math3d.add(
      point, Vector3(0, 0, constants.GRINDER_OFFSET - depth)
    )
    sequence.plan_to_z(trench_bottom.z)
    # grinding ice forward
    yaw_offset = 0 if parallel else -math.pi / 2
    trench_segment = math3d.scalar_multiply(
      length,
      Vector3(math.cos(yaw + yaw_offset), math.sin(yaw + yaw_offset), 0)
    )
    sequence.plan_linear_translation(trench_segment)
    # grind sideways
    if parallel:
      # NOTE: small angle approximation?
      sequence.plan_to_named_joint_translations(j_shou_yaw = 0.08)
    else:
      sequence.plan_to_translation(math3d.scalar_multiply(0.08,
          Vector3(math.cos(yaw), math.sin(yaw), 0)))
    # grind backwards towards lander
    sequence.plan_linear_translation(
      math3d.scalar_multiply(-1, trench_segment))
    # exit terrain
    sequence.plan_to_z(pregrind_position.z)
    return sequence.merge()

class TaskScoopCircularServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                              ActionServerBase):

  name          = 'TaskScoopCircular'
  action_type   = owl_msgs.msg.TaskScoopCircularAction
  goal_type     = owl_msgs.msg.TaskScoopCircularGoal
  feedback_type = owl_msgs.msg.TaskScoopCircularFeedback
  result_type   = owl_msgs.msg.TaskScoopCircularResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.TASK_GOAL

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def plan_trajectory(self, goal):
    # TODO:
    #  1. implement normal parameter
    #  2. implement scoop_angle parameter

    trench_surface = self.transform_to_planning_frame(
      self.get_intended_position(goal.frame, goal.relative, goal.point)).point
    trench_bottom = Point(
      trench_surface.x,
      trench_surface.y,
      trench_surface.z - goal.depth + constants.R_PARALLEL_FALSE_A
    )
    sequence = TrajectorySequence(
      self._arm.robot, self._arm.move_group_scoop, 'l_scoop')
    # place end-effector above trench position
    sequence.plan_to_named_joint_positions(
      j_shou_yaw = _compute_workspace_shoulder_yaw(
        trench_bottom.x, trench_bottom.y),
      j_shou_pitch = math.pi / 2,
      j_prox_pitch = -math.pi / 2,
      j_dist_pitch = 0.0,
      j_hand_yaw = 0.0,
      j_scoop_yaw = 0.0
    )
    if goal.parallel:
      # rotate hand so scoop bottom points down
      sequence.plan_to_named_joint_positions(j_hand_yaw = 0.0)
      # rotate scoop to face radially out from lander
      sequence.plan_to_named_joint_positions(j_scoop_yaw = math.pi/2)
      # pitch scoop back with the distal pitch so its blade faces terrain
      sequence.plan_to_named_joint_positions(j_dist_pitch = -19.0/54.0*math.pi)
      # Once aligned to trench goal, place hand above trench middle point
      sequence.plan_to_position(trench_bottom)
      # perform scoop by rotating distal pitch, and scoop through surface
      sequence.plan_to_named_joint_translations(j_dist_pitch = 2.0/3.0*math.pi)
    else:
      # lower to trench position, maintaining up-right orientation
      sequence.plan_to_position(trench_bottom)
      # rotate hand yaw so scoop tip points into surface
      sequence.plan_to_named_joint_positions(j_hand_yaw = math.pi/2.2)
      # lower scoop back to down z-position with new hand yaw position set
      sequence.plan_to_z(trench_bottom.z)
      # perform scoop by rotating hand yaw, and scoop through surface
      sequence.plan_to_named_joint_positions(j_hand_yaw = -0.29*math.pi)
    return sequence.merge()

class TaskScoopLinearServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                            ActionServerBase):

  name          = 'TaskScoopLinear'
  action_type   = owl_msgs.msg.TaskScoopLinearAction
  goal_type     = owl_msgs.msg.TaskScoopLinearGoal
  feedback_type = owl_msgs.msg.TaskScoopLinearFeedback
  result_type   = owl_msgs.msg.TaskScoopLinearResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.TASK_GOAL

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def plan_trajectory(self, goal):
    # TODO:
    #  1. implement normal parameter

    trench_surface = self.transform_to_planning_frame(
      self.get_intended_position(goal.frame, goal.relative, goal.point)).point
    ## TODO: comment on what these calculations do
    alpha = math.atan2(constants.WRIST_SCOOP_PARAL,
                       constants.WRIST_SCOOP_PERP)
    distance_from_ground = constants.ROT_RADIUS * \
      (math.cos(alpha) - math.sin(alpha))
    # TODO: what point does this reference??
    center = Point(
      trench_surface.x,
      trench_surface.y,
      trench_surface.z + constants.SCOOP_HEIGHT - goal.depth + distance_from_ground
    )
    sequence = TrajectorySequence(
      self._arm.robot, self._arm.move_group_scoop, 'l_scoop')
    # place end-effector above trench position
    sequence.plan_to_named_joint_positions(
      j_shou_yaw = _compute_workspace_shoulder_yaw(
        trench_surface.x, trench_surface.y),
      j_shou_pitch = math.pi / 2,
      j_prox_pitch = -math.pi / 2,
      j_dist_pitch = 0.0,
      j_hand_yaw = math.pi/2.2,
      j_scoop_yaw = 0.0
    )
    # rotate hand so scoop bottom points down
    sequence.plan_to_named_joint_positions(j_hand_yaw = 0.0)
    # rotate scoop to face radially out from lander
    sequence.plan_to_named_joint_positions(j_scoop_yaw = math.pi / 2)
    # retract scoop back so its blades face terrain
    sequence.plan_to_named_joint_positions(j_dist_pitch = -math.pi / 2)
    # place scoop at the trench side nearest to the lander
    sequence.plan_to_position(center)
    # TODO: what is this doing?
    sequence.plan_to_named_joint_positions(j_dist_pitch = 2.0/9.0 * math.pi)
    # compute the far end of the trench
    far_trench_pose = sequence.get_final_pose()
    _, _, yaw = math3d.euler_from_quaternion(far_trench_pose.orientation)
    far_trench_pose.position.x += goal.length * math.cos(yaw)
    far_trench_pose.position.y += goal.length * math.sin(yaw)
    # move the scoop along a linear path to the end of the trench
    sequence.plan_linear_path_to_pose(far_trench_pose)
    # pitch scoop upward to maintain sample
    sequence.plan_to_named_joint_positions(j_dist_pitch = math.pi / 2)
    return sequence.merge()


class TaskDiscardSampleServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                              ActionServerBase):

  name          = 'TaskDiscardSample'
  action_type   = owl_msgs.msg.TaskDiscardSampleAction
  goal_type     = owl_msgs.msg.TaskDiscardSampleGoal
  feedback_type = owl_msgs.msg.TaskDiscardSampleFeedback
  result_type   = owl_msgs.msg.TaskDiscardSampleResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.TASK_GOAL

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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.TASK_GOAL

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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.ARM_GOAL

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def publish_feedback_cb(self):
    self._publish_feedback(pose=self._arm_tip_monitor.get_link_pose())

  def execute_action(self, goal):
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
    except (ArmExecutionError, ArmPlanningError) as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        final_pose=self._arm_tip_monitor.get_link_pose())
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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.ARM_GOAL

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def execute_action(self, goal):
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
    except (ArmExecutionError, ArmPlanningError) as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        final_pose=self._arm_tip_monitor.get_link_pose(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque())
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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.ARM_GOAL

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
    # the normal vector direction the scoop's bottom faces in its frame
    SCOOP_DOWNWARD = Vector3(0, 0, 1)
    try:
      # transform only position
      intended_start_position_stamped = self.transform_to_planning_frame(
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
      header=intended_start_position_stamped.header,
      pose=Pose(
        position=intended_start_position_stamped.point,
        orientation=orientation
      )
    )
    max_distance = goal.distance + goal.overdrive
    displacement = math3d.scalar_multiply(max_distance, normal)
    # pose after end-effector has driven its maximum distance towards surface
    # if there is no surface, the end-effector will reach this pose
    intended_end_pose_stamped = PoseStamped(
      header=intended_start_position_stamped.header,
      pose=Pose(
        position=math3d.add(
          intended_start_position_stamped.point, displacement),
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
    except (ArmExecutionError, ArmPlanningError) as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err) + " - Setup trajectory failed",
        final_pose=self.get_end_effector_pose(constants.FRAME_ID_BASE).pose,
        final_distance=0, final_force=0, final_torque=0)
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
      d = math3d.subtract(pose.position, intended_start_position_stamped.point)
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
    except ArmExecutionError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err) + " - Surface approach trajectory failed",
        final_pose=self.get_end_effector_pose(constants.FRAME_ID_BASE).pose,
        final_distance=compute_distance(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque()
      )
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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.ARM_GOAL

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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.ARM_GOAL

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
  goal_group_id = owl_msgs.msg.ActionGoalStatus.ARM_GOAL

  # redefine execute_action to enable FT monitor
  def execute_action(self, goal):
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
    except (ArmPlanningError, ArmExecutionError) as err:
      self._arm.checkin_arm(self.name)
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
  goal_group_id = None

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


class CameraCaptureServer(ActionServerBase):

  name          = 'CameraCapture'
  action_type   = owl_msgs.msg.CameraCaptureAction
  goal_type     = owl_msgs.msg.CameraCaptureGoal
  feedback_type = owl_msgs.msg.CameraCaptureFeedback
  result_type   = owl_msgs.msg.CameraCaptureResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.CAMERA_GOAL

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

    # await point cloud or action preempt
    FREQUENCY = 10 # Hz
    TIMEOUT = 5   # seconds
    rate = rospy.Rate(FREQUENCY)
    for i in range(0, int(TIMEOUT * FREQUENCY)):
      # TODO: investigate what preempt's function is here and in other actions
      if self._is_preempt_requested():
        self._set_preempted("Action was preempted")
        return
      if self.point_cloud_created:
        self._set_succeeded("Point cloud received")
        return
      rate.sleep()
    self._set_aborted("Timed out waiting for point cloud")


class CameraSetExposureServer(ActionServerBase):

  name          = 'CameraSetExposure'
  action_type   = owl_msgs.msg.CameraSetExposureAction
  goal_type     = owl_msgs.msg.CameraSetExposureGoal
  feedback_type = owl_msgs.msg.CameraSetExposureFeedback
  result_type   = owl_msgs.msg.CameraSetExposureResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.CAMERA_GOAL

  def __init__(self):
    super(CameraSetExposureServer, self).__init__()
    # set up interface for setting camera exposure
    self._pub_exposure = rospy.Publisher('/gazebo/plugins/camera_sim/exposure',
                                         Float64,
                                         queue_size=10)
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
  goal_group_id = None

  def __init__(self):
    super(DockIngestSampleServer, self).__init__()
    # subscribe to contact sensor to know when sample dock has sample
    TOPIC_SAMPLE_DOCK_CONTACTS = "/ow_regolith/contacts/sample_dock"
    self._contacts_sub = rospy.Subscriber(
      TOPIC_SAMPLE_DOCK_CONTACTS,
      Contacts,
      self._on_dock_contacts
    )
    self._dock_contacts = list()
    self._ingesting = False # true when action is active
    self._last_contact_time = 0.0 # seconds
    self._remove_lock = False # true when dock contacts are being processed
    self._start_server()

  def _on_dock_contacts(self, msg):
    self._dock_contacts = msg.link_names
    if self._ingesting:
      # remove contacts if ingestion is active
      self._remove_dock_contacts()
      self._update_last_contact_time()

  def _update_last_contact_time(self):
    self._last_contact_time = rospy.get_time()

  def _remove_dock_contacts(self):
    # a lock is used to avoid redundant service calls to remove the same links
    if self._remove_lock:
      return
    self._remove_lock = True
    # do nothing if there are no contacts to remove
    if not self._dock_contacts:
      return
    # call the remove service for all dock contacts
    REMOVE_REGOLITH_SERVICE = '/ow_regolith/remove_regolith'
    rospy.wait_for_service(REMOVE_REGOLITH_SERVICE)
    try:
      service = rospy.ServiceProxy(REMOVE_REGOLITH_SERVICE, RemoveRegolith)
      result = service(self._dock_contacts)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s" % e)
      self._remove_service_failed = True
      return
    # mark sample as ingested so long as one dock contact was removed
    if result.success:
      self._sample_was_ingested = True
    else:
      self._remove_service_failed = True
    self._remove_lock = False

  def execute_action(self, _goal):
    self._ingesting = True
    self._sample_was_ingested = False
    self._remove_service_failed = False
    # remove sample already contacting the dock
    self._remove_dock_contacts()
    # start the no-sample timeout
    self._update_last_contact_time()
    # loop until there has been no sample in the dock for at least 3 seconds
    NO_CONTACTS_TIMEOUT = 3.0 # seconds
    loop_rate = rospy.Rate(0.2)
    while rospy.get_time() - self._last_contact_time < NO_CONTACTS_TIMEOUT:
      loop_rate.sleep()
    # cease ingestion and wrap up action
    self._ingesting = False
    self._remove_lock = False
    if self._remove_service_failed:
      self._set_aborted("Regolith removal service failed",
        sample_ingested=self._sample_was_ingested)
    else:
      message = "Sample ingested" if self._sample_was_ingested else \
                "No sample was present in dock"
      self._set_succeeded(message, sample_ingested=self._sample_was_ingested)


class PanTiltMoveJointsServer(mixins.PanTiltMoveMixin, ActionServerBase):

  name          = 'PanTiltMoveJoints'
  action_type   = owl_msgs.msg.PanTiltMoveJointsAction
  goal_type     = owl_msgs.msg.PanTiltMoveJointsGoal
  feedback_type = owl_msgs.msg.PanTiltMoveJointsFeedback
  result_type   = owl_msgs.msg.PanTiltMoveJointsResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.PAN_TILT_GOAL

  def publish_feedback_cb(self):
    self._publish_feedback(pan_position = self._pan_pos,
                           tilt_position = self._tilt_pos)

  def execute_action(self, goal):
    try:
      not_preempted = self.move_pan_and_tilt(goal.pan, goal.tilt)
    except ArmExecutionError as err:
      self._set_aborted(str(err),
        pan_position=self._pan_pos, tilt_position=self._tilt_pos)
    else:
      if not_preempted:
        self._set_succeeded("Reached commanded pan/tilt values",
          pan_position=self._pan_pos, tilt_position=self._tilt_pos)
      else:
        self._set_preempted("Action was preempted",
          pan_position=self._pan_pos, tilt_position=self._tilt_pos)


class PanServer(mixins.PanTiltMoveMixin, ActionServerBase):

  name          = 'Pan'
  action_type   = ow_lander.msg.PanAction
  goal_type     = ow_lander.msg.PanGoal
  feedback_type = ow_lander.msg.PanFeedback
  result_type   = ow_lander.msg.PanResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.PAN_TILT_GOAL

  def publish_feedback_cb(self):
    self._publish_feedback(pan_position = self._pan_pos)

  def execute_action(self, goal):
    try:
      not_preempted = self.move_pan(goal.pan)
    except ArmExecutionError as err:
      self._set_aborted(str(err), pan_position=self._pan_pos)
    else:
      if not_preempted:
        self._set_succeeded("Reached commanded pan value",
                            pan_position=self._pan_pos)
      else:
        self._set_preempted("Action was preempted",
                            pan_position=self._pan_pos)

class TiltServer(mixins.PanTiltMoveMixin, ActionServerBase):

  name          = 'Tilt'
  action_type   = ow_lander.msg.TiltAction
  goal_type     = ow_lander.msg.TiltGoal
  feedback_type = ow_lander.msg.TiltFeedback
  result_type   = ow_lander.msg.TiltResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.PAN_TILT_GOAL

  def publish_feedback_cb(self):
    self._publish_feedback(tilt_position = self._tilt_pos)

  def execute_action(self, goal):
    try:
      not_preempted = self.move_tilt(goal.tilt)
    except ArmExecutionError as err:
      self._set_aborted(str(err), tilt_position=self._tilt_pos)
    else:
      if not_preempted:
        self._set_succeeded("Reached commanded tilt value",
                            tilt_position=self._tilt_pos)
      else:
        self._set_preempted("Action was preempted",
                            tilt_position=self._tilt_pos)

class PanTiltMoveCartesianServer(mixins.PanTiltMoveMixin, ActionServerBase):

  name          = 'PanTiltMoveCartesian'
  action_type   = owl_msgs.msg.PanTiltMoveCartesianAction
  goal_type     = owl_msgs.msg.PanTiltMoveCartesianGoal
  feedback_type = owl_msgs.msg.PanTiltMoveCartesianFeedback
  result_type   = owl_msgs.msg.PanTiltMoveCartesianResult
  goal_group_id = owl_msgs.msg.ActionGoalStatus.PAN_TILT_GOAL

  def execute_action(self, goal):
    LOOKAT_FRAME = constants.FRAME_ID_BASE
    cam_center = FrameTransformer().lookup_transform(LOOKAT_FRAME,
                                                    'StereoCameraCenter_link')
    tilt_joint = FrameTransformer().lookup_transform(LOOKAT_FRAME,
                                                     'l_ant_panel')
    try:
      frame_id = mixins.FrameMixin.get_frame_id_from_index(goal.frame)
    except ArmExecutionError as err:
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
      not_preempted = self.move_pan_and_tilt(pan, tilt)
    except (AntennaPlanningError, AntennaExecutionError) as err:
      self._set_aborted(str(err))
    else:
      if not_preempted:
        self._set_succeeded("Reached commanded pan/tilt values")
      else:
        self._set_preempted("Action was preempted")
