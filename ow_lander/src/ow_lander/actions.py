# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines all lander actions"""

import rospy

import ow_lander.msg
import owl_msgs.msg
from ow_lander.server import ActionServerBase

# required for all arm actions
from ow_lander.mixins import *
# required for GuardedMove
from ow_lander.ground_detector import GroundDetector
from geometry_msgs.msg import Point
# required for ArmMoveCartesianGuarded
from ow_lander.ground_detector import FTSensorThresholdMonitor
# required for ArmFindSurface
from geometry_msgs.msg import Vector3, PoseStamped, Pose
from ow_lander import math3d
from ow_lander.frame_transformer import FrameTransformer

# required for LightSetIntensity
from irg_gazebo_plugins.msg import ShaderParamUpdate
# required for CameraCapture
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import PointCloud2
# required for DockIngestSample
from ow_regolith.srv import RemoveRegolith
from ow_regolith.msg import Contacts
# required for AntennaPanTilt
from ow_lander import constants
from ow_lander.common import (in_closed_range, radians_equivalent,
                              create_most_recent_header)
from sensor_msgs.msg import JointState

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

class StopServer(ArmActionMixin, ActionServerBase):

  name          = 'Stop'
  action_type   = ow_lander.msg.StopAction
  goal_type     = ow_lander.msg.StopGoal
  feedback_type = ow_lander.msg.StopFeedback
  result_type   = ow_lander.msg.StopResult

  def execute_action(self, _goal):
    if self._arm.stop_arm():
      self._set_succeeded("Arm trajectory stopped",
        final=self._arm_tip_monitor.get_link_position())
    else:
      self._set_aborted("No arm trajectory to stop",
        final=self._arm_tip_monitor.get_link_position())

### DEPRECATED: ArmFindSurface should be used in place of GuardedMove
class GuardedMoveServer(ArmActionMixin, ActionServerBase):

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


class ArmUnstowServer(ArmTrajectoryMixin, ActionServerBase):

  name          = 'ArmUnstow'
  action_type   = owl_msgs.msg.ArmUnstowAction
  goal_type     = owl_msgs.msg.ArmUnstowGoal
  feedback_type = owl_msgs.msg.ArmUnstowFeedback
  result_type   = owl_msgs.msg.ArmUnstowResult

  def plan_trajectory(self, _goal):
    return self._planner.plan_arm_to_target('arm_unstowed')


class ArmStowServer(ArmTrajectoryMixin, ActionServerBase):

  name          = 'ArmStow'
  action_type   = owl_msgs.msg.ArmStowAction
  goal_type     = owl_msgs.msg.ArmStowGoal
  feedback_type = owl_msgs.msg.ArmStowFeedback
  result_type   = owl_msgs.msg.ArmStowResult

  def plan_trajectory(self, _goal):
    return self._planner.plan_arm_to_target('arm_stowed')


class GrindServer(GrinderTrajectoryMixin, ActionServerBase):

  name          = 'Grind'
  action_type   = ow_lander.msg.GrindAction
  goal_type     = ow_lander.msg.GrindGoal
  feedback_type = ow_lander.msg.GrindFeedback
  result_type   = ow_lander.msg.GrindResult

  def plan_trajectory(self, goal):
    return self._planner.grind(goal)


class DigCircularServer(ArmTrajectoryMixinOld, ActionServerBase):

  name          = 'DigCircular'
  action_type   = ow_lander.msg.DigCircularAction
  goal_type     = ow_lander.msg.DigCircularGoal
  feedback_type = ow_lander.msg.DigCircularFeedback
  result_type   = ow_lander.msg.DigCircularResult

  def plan_trajectory(self, goal):
    return self._planner.dig_circular(goal)


class DigLinearServer(ArmTrajectoryMixinOld, ActionServerBase):

  name          = 'DigLinear'
  action_type   = ow_lander.msg.DigLinearAction
  goal_type     = ow_lander.msg.DigLinearGoal
  feedback_type = ow_lander.msg.DigLinearFeedback
  result_type   = ow_lander.msg.DigLinearResult

  def plan_trajectory(self, goal):
    return self._planner.dig_linear(goal)


class DiscardServer(ArmTrajectoryMixinOld, ActionServerBase):

  name          = 'Discard'
  action_type   = ow_lander.msg.DiscardAction
  goal_type     = ow_lander.msg.DiscardGoal
  feedback_type = ow_lander.msg.DiscardFeedback
  result_type   = ow_lander.msg.DiscardResult

  def plan_trajectory(self, goal):
    return self._planner.discard_sample(goal)


class DeliverServer(ArmTrajectoryMixinOld, ActionServerBase):

  name          = 'Deliver'
  action_type   = ow_lander.msg.DeliverAction
  goal_type     = ow_lander.msg.DeliverGoal
  feedback_type = ow_lander.msg.DeliverFeedback
  result_type   = ow_lander.msg.DeliverResult

  def plan_trajectory(self, _goal):
    return self._planner.deliver_sample()


class ArmMoveCartesianServer(ModifyPoseMixin,
                             ArmActionMixin,
                             ActionServerBase):

  name          = 'ArmMoveCartesian'
  action_type   = owl_msgs.msg.ArmMoveCartesianAction
  goal_type     = owl_msgs.msg.ArmMoveCartesianGoal
  feedback_type = owl_msgs.msg.ArmMoveCartesianFeedback
  result_type   = owl_msgs.msg.ArmMoveCartesianResult

  def publish_feedback_cb(self):
    self._publish_feedback(pose=self._arm_tip_monitor.get_link_pose())

  def execute_action(self, goal):
    frame_id = self.handle_frame_goal(goal)
    if frame_id is None:
      self._set_aborted(self.abort_message)
    pose = PoseStamped(
      header=create_most_recent_header(frame_id),
      pose=goal.pose
    )
    # perform action
    try:
      self._arm.checkout_arm(self.name)
      plan = self._planner.plan_arm_to_pose(pose, self.ARM_END_EFFECTOR)
      self._arm.execute_arm_trajectory(plan,
        action_feedback_cb=self.publish_feedback_cb)
    except RuntimeError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        final_pose=self._arm_tip_monitor.get_link_pose())
    else:
      self._arm.checkin_arm(self.name)
      if not self.pose_reached(pose):
        self._set_aborted(self.abort_message,
          final_pose=self._arm_tip_monitor.get_link_pose())
        return
      self._set_succeeded(f"{self.name} trajectory succeeded",
        final_pose=self._arm_tip_monitor.get_link_pose())


class ArmMoveCartesianGuardedServer(ModifyPoseMixin,
                                    ArmActionMixin,
                                    ActionServerBase):

  name          = 'ArmMoveCartesianGuarded'
  action_type   = owl_msgs.msg.ArmMoveCartesianGuardedAction
  goal_type     = owl_msgs.msg.ArmMoveCartesianGuardedGoal
  feedback_type = owl_msgs.msg.ArmMoveCartesianGuardedFeedback
  result_type   = owl_msgs.msg.ArmMoveCartesianGuardedResult

  def execute_action(self, goal):
    frame_id = self.handle_frame_goal(goal)
    if frame_id is None:
      self._set_aborted(self.abort_message)
    pose = PoseStamped(
      header=create_most_recent_header(frame_id),
      pose=goal.pose
    )
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
      plan = self._planner.plan_arm_to_pose(pose, self.ARM_END_EFFECTOR)
      self._arm.execute_arm_trajectory(plan, action_feedback_cb=guarded_cb)
    except RuntimeError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err),
        final_pose=self._arm_tip_monitor.get_link_pose(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque())
    else:
      self._arm.checkin_arm(self.name)
      if not monitor.threshold_breached() and not self.pose_reached(pose):
        # pose was not reached due to planning/monitor error
        # FIXME: this does not handle case where the pose check failed, better
        #        error handling is required
        self._set_aborted(
          "Arm failed to reach pose despite neither force nor torque " \
          "thresholds being breached. Likely the thresholds were set too " \
          "high or a planning error occurred. Try a lower threshold.",
          final_pose=self._arm_tip_monitor.get_link_pose(),
          final_force=monitor.get_force(),
          final_torque=monitor.get_torque()
        )
        return
      self._set_succeeded(
        _format_guarded_move_success_message(self.name, monitor),
        final_pose=self._arm_tip_monitor.get_link_pose(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque()
      )


class ArmFindSurfaceServer(ModifyPoseMixin, ArmActionMixin, ActionServerBase):

  name          = 'ArmFindSurface'
  action_type   = owl_msgs.msg.ArmFindSurfaceAction
  goal_type     = owl_msgs.msg.ArmFindSurfaceGoal
  feedback_type = owl_msgs.msg.ArmFindSurfaceFeedback
  result_type   = owl_msgs.msg.ArmFindSurfaceResult

  def publish_feedback_cb(self, distance=0, force=0, torque=0):
    self._publish_feedback(
      pose=self._planner.get_end_effector_pose(self.ARM_END_EFFECTOR).pose,
      distance=distance,
      force=force,
      torque=torque
    )

  def execute_action(self, goal):
    # the normal vector direction the scoop's bottom faces in its frame
    SCOOP_DOWNWARD = Vector3(0, 0, 1)
    frame_id = self.handle_frame_goal(goal)
    if frame_id is None:
      self._set_aborted(self.abort_message)
      return
    # orient scoop so that the bottom points in the opposite to the normal
    # NOTE: regardless of frame parameter orientation is in the base_link frame
    orientation = math3d.quaternion_rotation_between(SCOOP_DOWNWARD,
                                                     goal.normal)
    start = goal.position
    if frame_id == constants.FRAME_ID_TOOL:
      start = FrameTransformer().transform_present(start,
        constants.FRAME_ID_TOOL, constants.FRAME_ID_BASE)
      if start is None:
        self._set_aborted("Failed to perform necessary frame transforms for " \
                          "trajectory planning.")
        return
    max_distance = goal.distance + goal.overdrive
    displacement = math3d.scalar_multiply(max_distance, goal.normal)
    end = math3d.add(start, displacement)
    # pose before end-effector is driven towards surface
    pose1 = PoseStamped(
      header=create_most_recent_header(constants.FRAME_ID_BASE),
      pose=Pose(
        position=start,
        orientation=orientation
      )
    )
    # pose after end-effector has driven its maximum distance towards surface
    # if there is no surface, the end-effector will reach this pose
    pose2 = PoseStamped(
      header=create_most_recent_header(constants.FRAME_ID_BASE),
      pose=Pose(
        position=end,
        orientation=orientation
      )
    )
    # move to setup pose prior to surface approach
    try:
      self._arm.checkout_arm(self.name)
      plan1 = self._planner.plan_arm_to_pose(pose1, self.ARM_END_EFFECTOR)
      self._arm.execute_arm_trajectory(plan1,
        action_feedback_cb=self.publish_feedback_cb)
    except RuntimeError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err) + " - Setup trajectory failed",
        final_pose=self._planner.get_end_effector_pose(self.ARM_END_EFFECTOR).pose,
        final_distance=0, final_force=0, final_torque=0)
      return
    # TODO: pose1 should be verified here, but self.pose_reached does more than
    #       its name lets one, and should be redesigned. Left as future work
    # local function to compute progress of the action during surface approach
    def compute_distance():
      pose = self._planner.get_end_effector_pose(self.ARM_END_EFFECTOR).pose
      d = math3d.subtract(pose.position, start)
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
      plan2 = self._planner.plan_arm_to_pose(pose2, self.ARM_END_EFFECTOR)
      self._arm.execute_arm_trajectory(plan2, action_feedback_cb=guarded_cb)
    except RuntimeError as err:
      self._arm.checkin_arm(self.name)
      self._set_aborted(str(err) + " - Surface approach trajectory failed",
        final_pose=self._planner.get_end_effector_pose(self.ARM_END_EFFECTOR).pose,
        final_distance=compute_distance(),
        final_force=monitor.get_force(),
        final_torque=monitor.get_torque()
      )
    else:
      self._arm.checkin_arm(self.name)
      if not monitor.threshold_breached() and not self.pose_reached(pose2):
        # pose was not reached due to planning/monitor error
        # FIXME: this does not handle case where the pose check failed, better
        #        error handling is required
        self._set_aborted(
          "Arm failed to reach pose despite neither force nor torque " \
          "thresholds being breached. Likely the thresholds were set too " \
          "high or a planning error occurred. Try a lower threshold.",
          final_pose=self._planner.get_end_effector_pose(self.ARM_END_EFFECTOR).pose,
          final_distance=compute_distance(),
          final_force=monitor.get_force(),
          final_torque=monitor.get_torque()
        )
      elif not monitor.threshold_breached():
        self._set_succeeded(
          "No surface was found",
          final_pose=self._planner.get_end_effector_pose(self.ARM_END_EFFECTOR).pose,
          final_distance=compute_distance(),
          final_force=monitor.get_force(),
          final_torque=monitor.get_torque()
        )
      else:
        msg = _format_guarded_move_success_message(self.name, monitor)
        pose = self._planner.get_end_effector_pose(self.ARM_END_EFFECTOR).pose
        msg += f". Surface found at ({pose.position.x:0.3f}, "
        msg +=                     f"{pose.position.y:0.3f}, "
        msg +=                     f"{pose.position.z:0.3f})"
        self._set_succeeded(
          msg,
          final_pose=pose,
          final_distance=compute_distance(),
          final_force=monitor.get_force(),
          final_torque=monitor.get_torque()
        )


class ArmMoveJointServer(ModifyJointValuesMixin, ActionServerBase):

  name          = 'ArmMoveJoint'
  action_type   = ow_lander.msg.ArmMoveJointAction
  goal_type     = ow_lander.msg.ArmMoveJointGoal
  feedback_type = ow_lander.msg.ArmMoveJointFeedback
  result_type   = ow_lander.msg.ArmMoveJointResult

  def modify_joint_positions(self, goal):
    pos = self._arm_joints_monitor.get_joint_positions()
    if goal.joint < 0 or goal.joint >= len(pos):
      raise RuntimeError("Provided joint index is not within range")
    if goal.relative:
      pos[goal.joint] += goal.angle
    else:
      pos[goal.joint] = goal.angle
    return pos


class ArmMoveJointsServer(ModifyJointValuesMixin, ActionServerBase):

  name          = 'ArmMoveJoints'
  action_type   = ow_lander.msg.ArmMoveJointsAction
  goal_type     = ow_lander.msg.ArmMoveJointsGoal
  feedback_type = ow_lander.msg.ArmMoveJointsFeedback
  result_type   = ow_lander.msg.ArmMoveJointsResult

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


#############################
## NON-ARM RELATED ACTIONS
#############################

class LightSetIntensityServer(ActionServerBase):

  name          = 'LightSetIntensity'
  action_type   = ow_lander.msg.LightSetIntensityAction
  goal_type     = ow_lander.msg.LightSetIntensityGoal
  feedback_type = ow_lander.msg.LightSetIntensityFeedback
  result_type   = ow_lander.msg.LightSetIntensityResult

  def __init__(self):
    super(LightSetIntensityServer, self).__init__()
    # set up interface for changing mast light brightness
    self.light_pub = rospy.Publisher('/gazebo/global_shader_param',
                                     ShaderParamUpdate,
                                     queue_size=1)
    self.light_msg = ShaderParamUpdate()
    self.light_msg.shaderType = ShaderParamUpdate.SHADER_TYPE_FRAGMENT
    self._start_server()

  def execute_action(self, goal):
    intensity = goal.intensity
    name = goal.name.lower()
    # check intensity range
    if intensity < 0.0 or intensity > 1.0:
      msg = f"Intensity = {intensity} is out of range."
      # NOTE: This action's result format is redundant since actionlib already
      #       conveys both a "success" flag and a "message" string. This
      #       redundancy will be removed following command unification.
      self._set_aborted(msg, success=False, message=msg)
      return
    # check for correct names
    if name == 'left':
      self.light_msg.paramName = 'spotlightIntensityScale[0]'
    elif name == 'right':
      self.light_msg.paramName = 'spotlightIntensityScale[1]'
    else:
      msg = f"\'{name}\' is not a light indentifier."
      self._set_aborted(msg, success=False, message=msg)
      return
    self.light_msg.paramValue = str(goal.intensity)
    self.light_pub.publish(self.light_msg)
    msg = f"{name} light intensity set successfully."
    self._set_succeeded(msg, success=True, message=msg)


class CameraCaptureServer(ActionServerBase):

  name          = 'CameraCapture'
  action_type   = ow_lander.msg.CameraCaptureAction
  goal_type     = ow_lander.msg.CameraCaptureGoal
  feedback_type = ow_lander.msg.CameraCaptureFeedback
  result_type   = ow_lander.msg.CameraCaptureResult

  def __init__(self):
    super(CameraCaptureServer, self).__init__()
    # set up interface for capturing a photograph witht he camera
    self._pub_exposure = rospy.Publisher('/gazebo/plugins/camera_sim/exposure',
                                         Float64,
                                         queue_size=10)
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
    # Set exposure if it is specified
    if goal.exposure > 0:
      self._pub_exposure.publish(goal.exposure)
      # There is no guarantee that exposure is set before the image is triggered
      # Pause to make it highly likely that exposure is received first.
      rospy.sleep(1)

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


class AntennaPanTiltServer(ActionServerBase):

  name          = 'AntennaPanTiltAction'
  action_type   = ow_lander.msg.AntennaPanTiltAction
  goal_type     = ow_lander.msg.AntennaPanTiltGoal
  feedback_type = ow_lander.msg.AntennaPanTiltFeedback
  result_type   = ow_lander.msg.AntennaPanTiltResult

  JOINT_STATES_TOPIC = "/joint_states"

  def __init__(self):
    super(AntennaPanTiltServer, self).__init__()
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
    # position of pan and tlt of the lander is obtained from JointStates
    ANTENNA_PAN_JOINT = "j_ant_pan"
    ANTENNA_TILT_JOINT = "j_ant_tilt"
    try:
      id_pan = data.name.index(ANTENNA_PAN_JOINT)
      id_tilt = data.name.index(ANTENNA_TILT_JOINT)
    except ValueError as err:
      rospy.logerr_throttle(1,
        f"AntennaPanTiltServer: {err}; joint value missing in "\
        f"{self.JOINT_STATES_TOPIC} topic")
      return
    self._pan_pos = data.position[id_pan]
    self._tilt_pos = data.position[id_tilt]

  def execute_action(self, goal):
    # FIXME: tolerance should not be necessary once the float precision
    #        problem is fixed by command unification (OW-1085)
    if not in_closed_range(goal.pan,
        constants.PAN_MIN, constants.PAN_MAX,
        constants.PAN_TILT_INPUT_TOLERANCE):
      self._set_aborted(f"Requested pan {goal.pan} is not within allowed " \
                        f"limits and was rejected.",
                        pan_position = self._pan_pos,
                        tilt_position = self._tilt_pos)
      return
    if not in_closed_range(goal.tilt,
        constants.TILT_MIN, constants.TILT_MAX,
        constants.PAN_TILT_INPUT_TOLERANCE):
      self._set_aborted(f"Requested tilt {goal.tilt} is not within allowed " \
                        f"limits and was rejected.",
                        pan_position = self._pan_pos,
                        tilt_position = self._tilt_pos)
      return

    # publish requested values to start pan/tilt trajectory
    self._pan_pub.publish(goal.pan)
    self._tilt_pub.publish(goal.tilt)

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
    # loop until pan/tilt reach their goal values
    FREQUENCY = 1 # Hz
    TIMEOUT = 60 # seconds
    rate = rospy.Rate(FREQUENCY)
    for i in range(0, int(TIMEOUT * FREQUENCY)):
      if self._is_preempt_requested():
        self._set_preempted("Action was preempted",
          pan_position = self._pan_pos, tilt_position = self._tilt_pos)
        return
      # publish feedback message
      self._publish_feedback(pan_position = self._pan_pos,
                             tilt_position = self._tilt_pos)
      # check if joints have arrived at their goal values
      if (radians_equivalent(goal.pan, self._pan_pos, constants.PAN_TOLERANCE) and
          radians_equivalent(goal.tilt, self._tilt_pos, constants.TILT_TOLERANCE)):
        self._set_succeeded("Reached commanded pan/tilt values",
          pan_position=self._pan_pos, tilt_position=self._tilt_pos)
        return
      rate.sleep()
    self._set_aborted(
      "Timed out waiting for pan/tilt values to reach goal.",
      pan_position=self._pan_pos, tilt_position=self._tilt_pos
    )
