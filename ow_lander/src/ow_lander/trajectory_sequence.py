import rospy
import time
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose

from ow_lander import math3d
from ow_lander.common import create_header
from ow_lander.exception import ArmPlanningError

class TrajectorySequence:
  """Plan a sequence of trajectories for a given end-effector, robot, and move
  group.
  """

  SRV_COMPUTE_FK = '/compute_fk'

  def __init__(self, robot, move_group, end_effector=None):
    self._ee = end_effector
    self._robot = robot
    self._group = move_group
    self._joints_count = len(self._group.get_joints())
    self._sequence = list()
    self._most_recent_state = self._robot.get_current_state()
    self._most_recent_joint_positions = self._group.get_current_joint_values()
    self._planning_time_total = 0.0
    # initialize forward-kinematics facility
    SERVICE_TIMEOUT = 30 # seconds
    rospy.wait_for_service(self.SRV_COMPUTE_FK, SERVICE_TIMEOUT)
    self._compute_fk_srv = rospy.ServiceProxy(self.SRV_COMPUTE_FK,
                                              GetPositionFK)

  def __del__(self):
    # clean-up any target states left-over in move_group so they are not carried
    # over to unrelated planning
    self._group.clear_pose_targets()

  def _assert_end_effector_set(self):
    if self._ee is None:
      raise ArmPlanningError("End-effector must be provided to IK planning")

  def _assert_joint_index_validity(self, index):
    if index >= self._joints_count or index < 0:
      raise ArmPlanningError("Joint index is out of range")

  def _lookup_joint_index(self, joint_name):
    if joint_name not in self._group.get_joints():
      raise ArmPlanningError(f"Joint {joint_name} is unknown to the "
                          f"{self._group.get_name()} move group.")
    else:
      return self._group.get_joints().index(joint_name)

  def _compute_forward_kinematics(self, robot_state):
    # TODO unhandled exception could be raised
    self._assert_end_effector_set()
    result = self._compute_fk_srv(create_header('base_link'),# rospy.Time.now()),
                                  [self._ee], robot_state)
    if result.error_code.val != MoveItErrorCodes.SUCCESS:
      raise ArmPlanningError(f"{self.SRV_COMPUTE_FK} service returned "
                          f"error code {result.error_code}")
    return result.pose_stamped[0].pose

  def _get_final_joint_positions_of(self, trajectory):
    return trajectory.joint_trajectory.points[-1].positions

  def _get_final_robot_state_of(self, trajectory):
    assert(len(trajectory.joint_trajectory.points) > 0)
    rs = self._group.get_current_state()
    joint_states = self._get_final_joint_positions_of(trajectory)
    # TODO: is there not better way to do this???
    # adding antenna (0,0) and grinder positions (-0.1) which should not change
    rs.joint_state.position = (
        0, 0) + joint_states[:5] + (-0.1,) + (joint_states[5],)
    return rs

  def _append_trajectory(self, trajectory, planning_time):
    rospy.logdebug(f"Trajectory took {planning_time} seconds to plan.")
    self._sequence.append(trajectory)
    self._most_recent_state = self._get_final_robot_state_of(trajectory)
    self._most_recent_joint_positions \
      = list(self._get_final_joint_positions_of(trajectory))
    self._planning_time_total += planning_time

  def _plan(self):
    """Calls on MoveIt to plan the next trajectory of the sequence. If no
    trajectory is provided, the plan is constructed from
    """
    success, trajectory, planning_time, error_code = self._group.plan()
    if not success:
      raise ArmPlanningError(
        f"MoveIt planning failed with error code: {error_code}")
    self._append_trajectory(trajectory, planning_time)

  def _plan_to_joint_positions(self, joint_positions):
    """Plan for all joints to move to new configuration
    joint_positions -- list of arm joint positions in radians
    """
    if len(joint_positions) != self._joints_count:
      raise ArmPlanningError("Incorrect number of joints for arm move group")
    self._group.set_start_state(self._most_recent_state)
    self._group.set_joint_value_target(joint_positions)
    self._plan()

  def _plan_to_joint_translations(self, joint_translations):
    """Plan for all joints to change their positions by a list of translations
    joint_translations -- list of joint displacements in radians
    """
    if len(joint_translations) != self._joints_count:
      raise ArmPlanningError("Incorrect number of joints for arm move group")
    current = self._most_recent_joint_positions
    final = [x + y for x, y in zip(current, joint_translations)]
    self._plan_to_joint_positions(final)

  def _plan_to_coordinate(self, coordinate, position):
    """Internal helper function so position of a coordinate can be set
    independent of other coordinates and orientation.
    coordinate -- either the characters 'x', 'y', or 'z'
    position   -- absolute frame position coordinate will be moved to
    """
    if coordinate not in ['x', 'y', 'z']:
      raise ArmPlanningError(f"Unrecognized Cartesian coordinate {coordinate}")
    pose = self._compute_forward_kinematics(self._most_recent_state)
    setattr(pose.position, coordinate, position)
    self.plan_to_pose(pose)

  def plan_to_joint_positions(self, **kwargs):
    """Plan for one or more joints to move to a new position. Any joint not
    listed in kwargs will not change from its most recent position.
    kwargs -- keywords are joint names and their values are the joint's desired
              translation in radians
    """
    positions = self._most_recent_joint_positions
    for joint in kwargs:
      positions[self._lookup_joint_index(joint)] = kwargs[joint]
    self._plan_to_joint_positions(positions)

  def plan_to_joint_translations(self, **kwargs):
    """Plan for one or more joints to change their positions by some translation.
    Any joint not listed in kwargs will not translate from its most recent
    positions.
    kwargs -- keywords are joint names and their values are the joint's desired
              translation in radians
    """
    translations = [0.0] * self._joints_count
    for joint in kwargs:
        translations[self._lookup_joint_index(joint)] = kwargs[joint]
    self._plan_to_joint_translations(translations)

  def plan_to_target(self, target_name):
    """Plan to a named set of joint positions
    target_name -- named set of joint positions
    """
    self._plan_to_joint_positions(
      self._group.get_named_target_values(target_name))

  def plan_to_pose(self, pose):
    """Plan the end-effector to a new pose
    pose -- geometry_msgs Pose end-effector will move to
    """
    self._assert_end_effector_set()
    self._group.set_start_state(self._most_recent_state)
    self._group.set_pose_target(pose, self._ee)
    self._plan()

  def plan_linear_path_to_pose(self, pose):
    """Plan the end-effector along a linear path from its most recent pose in
    the sequence to a new pose
    pose -- geometry_msgs Pose
    """
    self._group.set_start_state(self._most_recent_state)
    start = time.time()
    trajectory, fraction = self._group.compute_cartesian_path(
      [pose], # sequence of waypoints
      0.01,   # end-effector follow step (meters)
      0.0     # jump threshold
    )
    planning_time = time.time() - start
    if fraction != 1.0:
      raise ArmPlanningError("Linear path planning failed")
    self._append_trajectory(trajectory, planning_time)

  def plan_linear_translation(self, translation):
    """Plan the end-effector along a linear path form its most recent pose in
    the sequence to a new pose, which will achieve the same orientation it
    started the trajectory with
    translation -- geometry_msgs Point/Vector3
    """
    current = self._compute_forward_kinematics(self._most_recent_state)
    final = Pose(
      math3d.add(current.position, translation),
      current.orientation
    )
    self.plan_linear_path_to_pose(final)

  def plan_to_position(self, point):
    """Plan the end-effector into a new position and acquire the same
    orientation in the final pose.
    translation -- geometry_msgs Point/Vector3 representing new position
    """
    current = self._compute_forward_kinematics(self._most_recent_state)
    final = Pose(point, current.orientation)
    self.plan_to_pose(final)

  def plan_to_orientation(self, orientation):
    """Plan the end-effector into a new orientation and acquire the same
    position in the final pose.
    orientation -- geometry_msgs Quaternion representing new orientation
    """
    current = self._compute_forward_kinematics(self._most_recent_state)
    final = Pose(current.position, orientation)
    self.plan_to_pose(final)

  def plan_to_translation(self, translation):
    """Plan the end-effector to move by a Cartesian translation and acquire the
    same orientation in the final pose.
    translation -- geometry_msgs Point/Vector3 representing a change in position
    """
    current = self._compute_forward_kinematics(self._most_recent_state)
    final = Pose(
      math3d.add(current.position, translation),
      current.orientation
    )
    self.plan_to_pose(final)

  def plan_to_x(self, position):
    """Set absolute x-position independent of other coordinates and orientation
    position -- New x-position
    """
    self._plan_to_coordinate('x', position)

  def plan_to_y(self, position):
    """Set absolute y-position independent of other coordinates and orientation
    position -- New y-position
    """
    self._plan_to_coordinate('y', position)

  def plan_to_z(self, position):
    """Set absolute z-position independent of other coordinates and orientation
    position -- New z-position
    """
    self._plan_to_coordinate('z', position)

  def get_final_joint_positions(self):
    return self._most_recent_joint_positions

  def get_final_pose(self):
    return self._compute_forward_kinematics(self._most_recent_state)

  def merge(self):
    """Merge all trajectories in the sequence into a single trajectory. Must be
    called after calling at least one `plan_to_*` method.
    returns a moveit_msgs/RobotTrajectory that is a merge of all contained
    trajectories
    """
    BETWEEN_TRAJECTORY_PAUSE = rospy.Duration(0.1)
    if len(self._sequence) == 0:
      raise ArmPlanningError("Sequence contains no trajectories")
    if len(self._sequence) == 1:
      return self._sequence[0]
    rospy.logdebug("Total time for planning the sequence was "
                   f"{self._planning_time_total} seconds")
    # merge trajectories together
    merged = JointTrajectory()
    merged.header.frame_id = self._group.get_pose_reference_frame()
    merged.joint_names = self._sequence[0].joint_trajectory.joint_names
    time_offset = rospy.Duration(0)
    for trajectory in self._sequence:
      merged.points += [
        JointTrajectoryPoint(
          x.positions, x.velocities, x.accelerations, x.effort,
          x.time_from_start + time_offset
        ) for x in trajectory.joint_trajectory.points
      ]
      # for next loop add total duration of this trajectory
      time_offset += trajectory.joint_trajectory.points[-1].time_from_start \
      # add a small pause so there are no points that overlap in time
      time_offset += BETWEEN_TRAJECTORY_PAUSE
    return RobotTrajectory(joint_trajectory = merged)
