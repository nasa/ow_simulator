import rospy
from moveit_msgs.srv import GetPositionFK, MoveItErrorCodes
from movie_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ow_lander.frame_transformer import FrameTransformer
from ow_lander.common import create_header

class PlanningException(RuntimeError):
    pass

class TrajectorySequence:

  SRV_COMPUTE_FK = '/compute_fk'

  def __init__(self, end_effector, robot, move_group):
    self._ee = end_effector
    self._robot = robot
    self._group = move_group
    self._sequence = list()
    self._most_recent_state = self._robot.get_current_state()
    # initialize forward-kinematics facility
    SERVICE_TIMEOUT = 30 # seconds
    rospy.wait_for_service(self.SRV_COMPUTE_FK, SERVICE_TIMEOUT)
    self._compute_fk_srv = rospy.ServiceProxy(self.SRV_COMPUTE_FK,
                                              GetPositionFK)

  def _get_final_robot_state_of(self, trajectory):
    assert(len(trajectory) > 0)
    rs = self._group.get_current_state()
    joint_states = trajectory.joint_trajectory.points[-1]
    # TODO: is there not better way to do this???
    # adding antenna (0,0) and grinder positions (-0.1) which should not change
    rs.joint_state.position = (
        0, 0) + joint_states[:5] + (-0.1,) + (joint_states[5],)
    return rs

  def _plan(self):
    success, trajectory, planning_time, error_code = self._group.plan()
    if not success:
      raise PlanningException(
        f"MoveIt planning failed with error code: {error_code}")
    rospy.logdebug(f"Plan took {planning_time} seconds.")
    self._sequence.append(trajectory)
    self._most_recent_state = self._get_final_robot_state_of(trajectory)

  def plan_to_joint_positions(self, joint_positions):
    """Construct a plan from an some initial configuration to the provided
    configuration
    joint_positions -- list of arm joint positions in radians
    returns
    """
    if len(joint_positions) != len(self._move_arm.get_joints()):
      raise PlanningException(
        "Incorrect number of joints for arm move group")
    self._group.set_start_state(self._most_recent_state)
    self._group.set_joint_value_target(joint_positions)
    self._plan()

  def plan_to_target(self, target_name):
    self.plan_to_joint_positions(
      self._group.get_named_target_values(target_name))

  def plan_to_pose(self, pose):
    """Plan a trajectory from arm's current pose to a new pose
    pose         -- Stamped pose plan will place end-effector at
    end_effector -- Name of end_effector
    """
    group_frame = self._group.get_pose_reference_frame()
    pose_t = FrameTransformer().transform(pose, group_frame)
    if pose_t is None:
      raise PlanningException(
        "Failed to transform requested pose to pose reference frame")
    self._group.set_start_state(self._most_recent_state)
    self._group.set_pose_target(pose, self._ee)
    self._plan()
    # TODO: investigate if this is necessary
    self._group.clear_pose_target(self._ee)

  def compute_forward_kinematics(self, fk_target_links, robot_state):
    result = self._compute_fk_srv(create_header('base_link', rospy.Time.now()),
                                  fk_target_links, robot_state)
    if result.error_code != MoveItErrorCodes.SUCCESS:
      raise PlanningException(f"{self.SRV_COMPUTE_FK} service returned "
                              f"error code {result.error_code}")
    return result.pose_stamped

  def merge(self):
    if len(self._sequence) == 0:
      raise PlanningException("Sequence contains no trajectories")
    if len(self._sequence) == 1:
      return self._sequence[0]
    BETWEEN_TRAJECTORY_PAUSE = rospy.Duration(0.1)
    # merge trajectories together
    merged = JointTrajectory()
    merged.header.frame_id = self._sequence[0].joint_trajectory.header
    merged.joint_names = self._sequence[0].joint_trajectory.joint_names
    time_offset = rospy.Duration(0)
    for trajectory in self._sequence:
      merged.points += [
        JointTrajectoryPoint(
          x.positions, x.velocities, x.accelerations,
          x.time_from_start + time_offset
        ) for x in trajectory.joint_trajectory.points
      ]
      # for next loop add total duration of this trajectory
      time_offset += trajectory.joint_trajectory.points[-1].time_from_start \
      # add a small pause so there are no points that overlap in time
      time_offset += BETWEEN_TRAJECTORY_PAUSE
    return RobotTrajectory(joint_trajectory = merged)
