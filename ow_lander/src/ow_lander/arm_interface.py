# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines a trajectory execution interface to the OceanWATERS arm. Integrates
with the OceanWATERS fault system to enable arm fault awareness and enables the
ability to stop the arm mid-trajectory.
"""

import rospy
import actionlib
import moveit_commander
import dynamic_reconfigure.client
from owl_msgs.msg import SystemFaultsStatus
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from actionlib_msgs.msg import GoalStatus
from controller_manager_msgs.srv import SwitchController

from ow_lander.common import Singleton
from ow_lander.exception import ArmExecutionError
from ow_lander.frame_transformer import FrameTransformer

class OWArmInterface(metaclass = Singleton):
  """Implements an ownership layer and stop method over trajectory execution.
  Ownership is claimed/relinquished via the check out/in methods.
  """

  """A string that identifies what facility is using the arm. If None, arm is
  not in use.
  """
  _in_use_by = None

  """True if arm is checked out and stop_arm was called. Set to false when arm
  is checked in.
  """
  _stopped = False

  @classmethod
  def checkout_arm(cls, owner):
    if cls._in_use_by is not None:
      raise ArmExecutionError(
        f"Arm is already checked out by the {cls._in_use_by} action server")
    cls._in_use_by = owner

  @classmethod
  def checkin_arm(cls, owner):
    if cls._in_use_by != owner:
      return # owner has not checked out arm, do nothing
    cls._in_use_by = None
    cls._stopped = False

  @classmethod
  def stop_arm(cls):
    if cls._in_use_by:
      cls._stopped = True
    return cls._stopped

  @classmethod
  def _assert_arm_is_checked_out(cls):
    if cls._in_use_by is None:
      raise ArmExecutionError("Arm action did not check-out the arm before " \
                         "attempting to execute a trajectory.")

  def __init__(self):
    # initialize/reference trajectory execution singleton
    self.__executor = ArmTrajectoryExecutor()
    # initialize/reference fault monitor
    self.__faults = ArmFaultMonitor()
    # basic MoveIt interface for arm planning
    self.robot = moveit_commander.RobotCommander()
    self.move_group_scoop = moveit_commander.MoveGroupCommander('arm')
    self.move_group_grinder = moveit_commander.MoveGroupCommander('grinder')

  def _stop_arm_if_fault(self, _feedback=None):
    """Ticks the stop flag when arm should stop due to a fault."""
    if not self.__faults.should_arm_continue_in_fault() and \
        self.__faults.is_arm_faulted():
      OWArmInterface._stopped = True

  def stop_trajectory_silently(self):
    """Will bypass the stop flag and cease trajectory execution directly. This
    results in no exception being thrown."""
    self.__executor.cease_execution()

  def switch_to_grinder_controller(self):
    OWArmInterface._assert_arm_is_checked_out()
    if self.__executor.get_active_controller() == 'grinder_controller':
      return
    if not self.__executor.switch_controllers('grinder_controller',
                                             'arm_controller'):
      raise ArmExecutionError("Failed to switch to grinder_controller")

  def switch_to_arm_controller(self):
    OWArmInterface._assert_arm_is_checked_out()
    if self.__executor.get_active_controller() == 'arm_controller':
      return
    if not self.__executor.switch_controllers('arm_controller',
                                             'grinder_controller'):
      raise ArmExecutionError("Failed to switch to arm_controller")

  def execute_arm_trajectory(self, plan, action_feedback_cb=None):
    """Executes the provided plan and awaits its completions
    plan -- An instance of moveit_msgs.msg.RobotTrajectory that describes the
            arm trajectory to be executed. Can be None, in which case planning
            is assumed to have failed.
    action_feedback_cb -- A function called at 100 Hz during execution of a
                          trajectory. Exists to publish the action's feedback
                          message. Handles no arguments.
    returns True if plan was executed successfully and without preempt.
    """

    OWArmInterface._assert_arm_is_checked_out()

    if not plan or len(plan.joint_trajectory.points) == 0:
      # trajectory planner returns false when planning has failed
      # other planning functions may simply return an empty plan
      raise ArmExecutionError("Trajectory planning failed")

    # check if fault occurred during planning phase
    self._stop_arm_if_fault()

    if OWArmInterface._stopped:
      raise ArmExecutionError("Stop was called; trajectory will not be executed")

    self.__executor.execute(plan.joint_trajectory,
      feedback_cb=self._stop_arm_if_fault)

    # publish feedback while waiting for trajectory execution completion
    FEEDBACK_RATE = 100 # hertz
    rate = rospy.Rate(FEEDBACK_RATE)
    timeout = plan.joint_trajectory.points[-1].time_from_start \
              - plan.joint_trajectory.points[0].time_from_start
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < timeout.to_sec() \
          or self.__executor.is_active():
      if OWArmInterface._stopped:
        self.__executor.cease_execution()
        raise ArmExecutionError("Stop was called; trajectory execution ceased")
      if action_feedback_cb is not None:
        action_feedback_cb()
      rate.sleep()

  def get_end_effector_pose(self, end_effector, frame_id,
                            timestamp=rospy.Time(0), timeout=rospy.Duration(0)):
    """Look up the pose of an end-effector in the currently active control group
    end_effector -- Name of the end-effector link
    frame_id     -- Frame ID in which to provide the result
    timestamp    -- See method comments in frame_transformer.py for usage
                    default: rospy.Time(0)
    timeout      -- See method comments in frame_tansformer.py for usage
                    default: rospy.Duration(0)
    returns geometry_msgs.PoseStamped
    """
    pose = self._move_arm.get_current_pose(end_effector)
    pose.header.stamp = timestamp
    return FrameTransformer().transform(pose, frame_id, timeout)

class ArmFaultMonitor(metaclass=Singleton):
    """Checks whether the arm has faulted."""

    CONTINUE_ARM_IN_FAULT_NAME = 'arm_motion_continues_in_fault'

    def __init__(self):
        """Subscribes to system_fault_status for arm fault updates and connects
        to the faults dynamic reconfigure server to retrieve the continue arm in
        fault flag.
        May raise TimeoutError if a server connection fails.
        """
        SYSTEM__FAULTS_TOPIC = "/system__faults_status"
        self._sub_system__faults = rospy.Subscriber(
            SYSTEM__FAULTS_TOPIC,
            SystemFaultsStatus,
            self._system__faults_callback
        )
        self._arm_fault = False

        DYNRECON_CLIENT_TIMEOUT = 30 # seconds
        DYNRECON_NAME = "faults"
        self.__faults_reconfigure_client = dynamic_reconfigure.client.Client(
            DYNRECON_NAME, timeout=DYNRECON_CLIENT_TIMEOUT,
            config_callback=self._dynamic_reconfigure_callback)
        config = self.__faults_reconfigure_client.get_configuration(
            DYNRECON_CLIENT_TIMEOUT)
        if not config:
            raise TimeoutError(
                f"Timed out waiting {DYNRECON_CLIENT_TIMEOUT} seconds for a " \
                f"connection with the {DYNRECON_NAME} dynamic reconfigure " \
                f"server."
            )
        rospy.loginfo(
            "Successfully connected to the dynamic reconfigure server.")
        self._continue_arm_in_fault = config[
            ArmFaultMonitor.CONTINUE_ARM_IN_FAULT_NAME]

    def _system__faults_callback(self, data):
        """If system fault occurs and it is an arm failure, mark flag true"""
        ARM_FAULT = 4
        self._arm_fault = data.value & ARM_FAULT == ARM_FAULT

    def _dynamic_reconfigure_callback(self, config):
        """Updates the continue arm flag from dynamic reconfigure server."""
        self._continue_arm_in_fault = config[
            ArmFaultMonitor.CONTINUE_ARM_IN_FAULT_NAME]
        # TODO: could this callback also assign arm_fault?

    def is_arm_faulted(self):
        return self._arm_fault

    def should_arm_continue_in_fault(self):
        return self._continue_arm_in_fault

class ArmTrajectoryExecutor(metaclass=Singleton):
    """Invokes the follow_joint_trajectory actions of arm controllers to
    execute a trajectory provided as a moveit_msgs.msg.RobotTrajectory.
    Execution occurs asynchronously and can be ceased with a method call.
    """

    def __init__(self):
        """Connect to action server of specified controller.
        May raise a TimeoutError if a server connection fails.
        """
        # initialize action client for all required controllers (arm and grinder)
        ACTION_CLIENT_TIMEOUT = 30 # seconds
        self.SUPPORTED_CONTROLLERS = ['arm_controller', 'grinder_controller']
        self._active_controller = self.SUPPORTED_CONTROLLERS[0]
        self._follow_action_clients = dict()
        for controller in self.SUPPORTED_CONTROLLERS:
            action = f"{controller}/follow_joint_trajectory"
            self._follow_action_clients[controller] = actionlib. \
              SimpleActionClient(action, FollowJointTrajectoryAction)
            if not self._follow_action_clients[controller].wait_for_server(
                    rospy.Duration(ACTION_CLIENT_TIMEOUT)):
                raise TimeoutError(
                  f"Timed out waiting {ACTION_CLIENT_TIMEOUT} seconds for "\
                  f"connection to the {action} action server."
                )
            rospy.loginfo(
              f"Successfully connected to the {action} action server.")
        # initialize controller switch service proxy to enable grinder trajectories
        SWITCH_CONTROLLER_SERVICE = '/controller_manager/switch_controller'
        SERVICE_PROXY_TIMEOUT = 30 # seconds
        rospy.wait_for_service(SWITCH_CONTROLLER_SERVICE, SERVICE_PROXY_TIMEOUT)
        self._switch_controller_srv = rospy.ServiceProxy(
          SWITCH_CONTROLLER_SERVICE, SwitchController)

    def _get_active_follow_client(self):
        return self._follow_action_clients[self._active_controller]

    def switch_controllers(self, switch_to, switch_from):
        if not switch_to in self.SUPPORTED_CONTROLLERS:
            rospy.logerr(f"{switch_to} is not a supported controller")
            return False
        if switch_to == self._active_controller:
            return False
        success = False
        try:
            success = self._switch_controller_srv(
                [switch_to], [switch_from], 2, False, 1.0)
        except rospy.ServiceException as err:
            rospy.loginfo(f"Failed to call switch_controller service: {err}")
        # This sleep is a workaround for "start point deviates from current
        # robot state" error on dig_circular trajectory execution.
        if success:
            self._active_controller = switch_to
            rospy.sleep(0.2)
        return success

    def get_active_controller(self):
        """Returns the name of the currently active joint controller
        """
        return self._active_controller

    def execute(self, trajectory,
                done_cb=None, active_cb=None, feedback_cb=None):
        """Execute the provided trajectory asynchronously.
        trajectory -- An instance of moveit_msgs.msg.RobotTrajectory that
                      describes the desired trajectory.
        """
        goal = FollowJointTrajectoryGoal(trajectory=trajectory)
        self._get_active_follow_client().send_goal(
            goal, done_cb, active_cb, feedback_cb)
        # block until client is active, which should only take some milliseconds
        CHECK_RATE = 500 # hertz
        MAX_WAIT = 1.0 # seconds
        rate = rospy.Rate(CHECK_RATE)
        for i in range(int(CHECK_RATE * MAX_WAIT)):
          if self.is_active():
            return
          rate.sleep()
        rospy.logwarn(f"The {self._active_controller}/follow_joint_trajectory "
                      f"action failed to become active within {MAX_WAIT} "
                      "seconds of sending a goal.")

    def cease_execution(self):
        """Stops the execution of the last trajectory submitted for execution"""
        if self._get_active_follow_client().get_state() == GoalStatus.ACTIVE:
            self._get_active_follow_client().cancel_goal()

    def wait(self, timeout=0):
        """Blocks until the execution of the current trajectory comes to an end
        timeout -- Seconds to wait for results
        """
        self._get_active_follow_client().wait_for_result(
            timeout=rospy.Duration(timeout))

    def result(self):
        """Gets the result of the last goal
        """
        return self._get_active_follow_client().get_result()

    def is_active(self):
        """Returns true if a trajectory is being executed and has not resulted
        in an error code.
        """
        return self._get_active_follow_client().get_state() == GoalStatus.ACTIVE
