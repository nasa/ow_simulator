# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines all lander actions"""

import math

import rospy
import owl_msgs.msg
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3, PoseStamped, Pose
from ow_regolith.srv import RemoveRegolith
from ow_regolith.msg import Contacts

import ow_lander.msg
from ow_lander import mixins
from ow_lander import math3d
from ow_lander import constants
from ow_lander.server import ActionServerBase
from ow_lander.common import normalize_radians
from ow_lander.ground_detector import GroundDetector, FTSensorThresholdMonitor
from ow_lander.frame_transformer import FrameTransformer

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


#####################
## ARM ACTIONS
#####################

class ArmStopServer(mixins.ArmActionMixin, ActionServerBase):

  name          = 'ArmStop'
  action_type   = owl_msgs.msg.ArmStopAction
  goal_type     = owl_msgs.msg.ArmStopGoal
  feedback_type = owl_msgs.msg.ArmStopFeedback
  result_type   = owl_msgs.msg.ArmStopResult

  def execute_action(self, _goal):
    if self._arm.stop_arm():
      self._set_succeeded("Arm trajectory stopped",
        final=self._arm_tip_monitor.get_link_position())
    else:
      self._set_aborted("No arm trajectory to stop",
        final=self._arm_tip_monitor.get_link_position())

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

    try:
      self._arm.checkout_arm(self.name)
      # TODO: split guarded_move trajectory into 2 parts so that ground
      #       detection can be started before the second execute_trajectory is
      #       called
      plan = self._planner.guarded_move(goal)
      self._arm.execute_arm_trajectory(plan,
        action_feedback_cb=ground_detect_cb)
    except RuntimeError as err:
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


class ArmUnstowServer(mixins.ArmTrajectoryMixin, ActionServerBase):

  name          = 'ArmUnstow'
  action_type   = owl_msgs.msg.ArmUnstowAction
  goal_type     = owl_msgs.msg.ArmUnstowGoal
  feedback_type = owl_msgs.msg.ArmUnstowFeedback
  result_type   = owl_msgs.msg.ArmUnstowResult

  def plan_trajectory(self, _goal):
    return self._planner.plan_arm_to_target('arm_unstowed')


class ArmStowServer(mixins.ArmTrajectoryMixin, ActionServerBase):

  name          = 'ArmStow'
  action_type   = owl_msgs.msg.ArmStowAction
  goal_type     = owl_msgs.msg.ArmStowGoal
  feedback_type = owl_msgs.msg.ArmStowFeedback
  result_type   = owl_msgs.msg.ArmStowResult

  def plan_trajectory(self, _goal):
    return self._planner.plan_arm_to_target('arm_stowed')


class TaskGrindServer(mixins.GrinderTrajectoryMixin, ActionServerBase):

  name          = 'TaskGrind'
  action_type   = owl_msgs.msg.TaskGrindAction
  goal_type     = owl_msgs.msg.TaskGrindGoal
  feedback_type = owl_msgs.msg.TaskGrindFeedback
  result_type   = owl_msgs.msg.TaskGrindResult

  def plan_trajectory(self, goal):
    return self._planner.grind(goal)


class TaskScoopCircularServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                              ActionServerBase):

  name          = 'TaskScoopCircular'
  action_type   = owl_msgs.msg.TaskScoopCircularAction
  goal_type     = owl_msgs.msg.TaskScoopCircularGoal
  feedback_type = owl_msgs.msg.TaskScoopCircularFeedback
  result_type   = owl_msgs.msg.TaskScoopCircularResult

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def plan_trajectory(self, goal):
    intended_dig_point = self.get_intended_position(
      goal.frame, goal.relative, goal.point)
    PLANNING_FRAME = 'base_link'
    point = FrameTransformer().transform(intended_dig_point, PLANNING_FRAME)
    if point is None:
      raise RuntimeError("Failed to transform dig point from " \
                         f"{goal.frame_id} to the {PLANNING_FRAME} frame")
    return self._planner.dig_circular(point.point, goal.depth, goal.parallel)


class TaskScoopLinearServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                            ActionServerBase):

  name          = 'TaskScoopLinear'
  action_type   = owl_msgs.msg.TaskScoopLinearAction
  goal_type     = owl_msgs.msg.TaskScoopLinearGoal
  feedback_type = owl_msgs.msg.TaskScoopLinearFeedback
  result_type   = owl_msgs.msg.TaskScoopLinearResult

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def plan_trajectory(self, goal):
    intended_dig_point = self.get_intended_position(
      goal.frame, goal.relative, goal.point)
    PLANNING_FRAME = 'base_link'
    point = FrameTransformer().transform(intended_dig_point, PLANNING_FRAME)
    if point is None:
      raise RuntimeError("Failed to transform dig point from " \
                         f"{goal.frame_id} to the {PLANNING_FRAME} frame")
    return self._planner.dig_linear(point.point, goal.depth, goal.length)


class TaskDiscardSampleServer(mixins.FrameMixin, mixins.ArmTrajectoryMixin,
                              ActionServerBase):

  name          = 'TaskDiscardSample'
  action_type   = owl_msgs.msg.TaskDiscardSampleAction
  goal_type     = owl_msgs.msg.TaskDiscardSampleGoal
  feedback_type = owl_msgs.msg.TaskDiscardSampleFeedback
  result_type   = owl_msgs.msg.TaskDiscardSampleResult

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def plan_trajectory(self, goal):
    intended_discard_point = self.get_intended_position(
      goal.frame, goal.relative, goal.point)
    PLANNING_FRAME = 'base_link'
    point = FrameTransformer().transform(intended_discard_point, PLANNING_FRAME)
    if point is None:
      raise RuntimeError("Failed to transform dig point from " \
                         f"{goal.frame_id} to the {PLANNING_FRAME} frame")
    return self._planner.discard_sample(point.point, goal.height)


class TaskDeliverSampleServer(mixins.ArmTrajectoryMixin, ActionServerBase):

  name          = 'TaskDeliverSample'
  action_type   = owl_msgs.msg.TaskDeliverSampleAction
  goal_type     = owl_msgs.msg.TaskDeliverSampleGoal
  feedback_type = owl_msgs.msg.TaskDeliverSampleFeedback
  result_type   = owl_msgs.msg.TaskDeliverSampleResult

  def plan_trajectory(self, _goal):
    return self._planner.deliver_sample()


class ArmMoveCartesianServer(mixins.FrameMixin, mixins.ArmActionMixin,
                             ActionServerBase):

  name          = 'ArmMoveCartesian'
  action_type   = owl_msgs.msg.ArmMoveCartesianAction
  goal_type     = owl_msgs.msg.ArmMoveCartesianGoal
  feedback_type = owl_msgs.msg.ArmMoveCartesianFeedback
  result_type   = owl_msgs.msg.ArmMoveCartesianResult

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def publish_feedback_cb(self):
    self._publish_feedback(pose=self._arm_tip_monitor.get_link_pose())

  def execute_action(self, goal):
    try:
      intended_pose_stamped = self.get_intended_pose(goal.frame, goal.relative,
                                                     goal.pose)
    except RuntimeError as err:
      self._set_aborted(str(err))
      return
    try:
      self._arm.checkout_arm(self.name)
      plan = self.plan_end_effector_to_pose(intended_pose_stamped)
      comparison_transform = self.get_comparison_transform(
        intended_pose_stamped.header.frame_id)
      self._arm.execute_arm_trajectory(plan,
        action_feedback_cb=self.publish_feedback_cb)
    except RuntimeError as err:
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

  def __init__(self, *args, **kwargs):
    super().__init__('l_scoop_tip', *args, **kwargs)

  def execute_action(self, goal):
    try:
      intended_pose_stamped = self.get_intended_pose(goal.frame, goal.relative,
                                                     goal.pose)
    except RuntimeError as err:
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
    except RuntimeError as err:
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
      # pre-transform only position
      intended_start_position_stamped = FrameTransformer().transform(
        self.get_intended_position(goal.frame, goal.relative, goal.position),
        constants.FRAME_ID_BASE
      )
      # normal is always considered in base_link regardless of frame parameter
      normal = self.validate_normalization(goal.normal)
    except RuntimeError as err:
      self._set_aborted(str(err))
      return
    if intended_start_position_stamped is None:
      self._set_aborted("Failed to perform necessary transform of position.")
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
      plan1 = self.plan_end_effector_to_pose(intended_start_pose_stamped)
      comparison_transform = self.get_comparison_transform(
        intended_start_pose_stamped.header.frame_id)
      self._arm.execute_arm_trajectory(plan1,
        action_feedback_cb=self.publish_feedback_cb)
    except RuntimeError as err:
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
      plan2 = self.plan_end_effector_to_pose(intended_end_pose_stamped)
      comparison_transform = self.get_comparison_transform(
        intended_start_pose_stamped.header.frame_id)
      self._arm.execute_arm_trajectory(plan2, action_feedback_cb=guarded_cb)
    except RuntimeError as err:
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

  def modify_joint_positions(self, goal):
    pos = self._arm_joints_monitor.get_joint_positions()
    if goal.joint < 0 or goal.joint >= len(pos):
      raise RuntimeError("Provided joint index is not within range")
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

  def modify_joint_positions(self, goal):
    pos = self._arm_joints_monitor.get_joint_positions()
    if len(goal.angles) != len(pos):
      raise RuntimeError("Number of angle positions provided does not much " \
                         "the number of joints in the arm move group")
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
      plan = self._planner.plan_arm_to_joint_angles(new_positions)
      self._arm.execute_arm_trajectory(plan, action_feedback_cb=guarded_cb)
    except RuntimeError as err:
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

  def publish_feedback_cb(self):
    self._publish_feedback(pan_position = self._pan_pos,
                           tilt_position = self._tilt_pos)

  def execute_action(self, goal):
    try:
      not_preempted = self.move_pan_and_tilt(goal.pan, goal.tilt)
    except RuntimeError as err:
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

  def publish_feedback_cb(self):
    self._publish_feedback(pan_position = self._pan_pos)

  def execute_action(self, goal):
    try:
      not_preempted = self.move_pan(goal.pan)
    except RuntimeError as err:
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

  def publish_feedback_cb(self):
    self._publish_feedback(tilt_position = self._tilt_pos)

  def execute_action(self, goal):
    try:
      not_preempted = self.move_tilt(goal.tilt)
    except RuntimeError as err:
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

  def execute_action(self, goal):
    LOOKAT_FRAME = constants.FRAME_ID_BASE
    cam_center = FrameTransformer().lookup_transform(LOOKAT_FRAME,
                                                    'StereoCameraCenter_link')
    tilt_joint = FrameTransformer().lookup_transform(LOOKAT_FRAME,
                                                     'l_ant_panel')
    try:
      frame_id = mixins.FrameMixin.get_frame_id_from_index(goal.frame)
    except RuntimeError as err:
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
    except RuntimeError as err:
      self._set_aborted(str(err))
    else:
      if not_preempted:
        self._set_succeeded("Reached commanded pan/tilt values")
      else:
        self._set_preempted("Action was preempted")
