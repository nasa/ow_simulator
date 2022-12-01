#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import dynamic_reconfigure.client
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from owl_msgs.msg import SystemFaultsStatus

from ow_actions.common import Singleton

ARM_EXECUTION_ERROR = 4

class ArmTrajectoryExecutor(metaclass=Singleton):
  """
  Interfaces with a Joint Trajectory Action Server of a given controller to
  execute a precomputed trajectory.
  """

  def __init__(self):
    """
    Connects to action server of specified controller and the dynamic
    reconfigure server. If either connection fails false is returned.
    :returns: True if ground connection was established, False otherwise
    :rtype: boolean
    """
    # subscribe to system_fault_status for any arm faults
    SYSTEM_FAULTS_TOPIC = "/system_faults_status"
    self._sub_system_faults = rospy.Subscriber(
      SYSTEM_FAULTS_TOPIC, SystemFaultsStatus, self._system_faults_callback)

    # intialize a dynamic reconfigure client to receive reconfigurations
    DYNRECON_CLIENT_TIMEOUT = 30 # seconds
    DYNRECON_NAME = "faults"
    self._faults_reconfigure_client = dynamic_reconfigure.client.Client(
      DYNRECON_NAME, timeout=DYNRECON_CLIENT_TIMEOUT,
      config_callback=self._dyanmic_reconfigure_callback)
    config = self._faults_reconfigure_client.get_configuration(
      DYNRECON_CLIENT_TIMEOUT)
    if not config:
      raise TimeoutError(
        f"Timed out waiting {DYNRECON_CLIENT_TIMEOUT} seconds for a " \
        f"connection with the {DYNRECON_NAME} dynamic reconfigure server."
      )
    rospy.loginfo("Successfully connected to the dyanmic reconfigure server.")
    self.arm_motion_continues_in_fault = config['arm_motion_continues_in_fault']

    # initialize an action client to send joint trajectories
    ACTION_CLIENT_TIMEOUT = 30 # seconds
    ARM_CONTROLLER_NAME = 'arm_controller'
    self._follow_action_client = actionlib.SimpleActionClient(
      f"{ARM_CONTROLLER_NAME}/follow_joint_trajectory",
      FollowJointTrajectoryAction
    )
    if not self._follow_action_client.wait_for_server(
        rospy.Duration(ACTION_CLIENT_TIMEOUT)):
      raise TimeoutError(
        f"Timed out waiting {ACTION_CLIENT_TIMEOUT} seconds for connection " \
        f"the {ARM_CONTROLLER_NAME} joint trajectory action server."
      )
    rospy.loginfo(f"Successfully connected to {ARM_CONTROLLER_NAME} joint "\
                  f"trajectory action server.")

  def _dyanmic_reconfigure_callback(self, config):
    """
    Update the corresponding flag from dynamic reconfigure server.
    """
    self.arm_motion_continues_in_fault = config['arm_motion_continues_in_fault']
    # TODO: couldn't this callback also assign arm_fault?

  def _system_faults_callback(self, data):
    """
    If system fault occurs and it is an arm failure, the arm_fault flag is set.
    """
    self.arm_fault = data.value & ARM_EXECUTION_ERROR == ARM_EXECUTION_ERROR

  def execute(self, trajectory, goal_time_tolerance=rospy.Time(0.1),
              done_cb=None, active_cb=None, feedback_cb=None):
    """
    :type trajectory: trajectory_msgs/JointTrajectory
    :type done_cb: function
    :type active_cb: function
    :type feedback_cb: function
    """
    if self.arm_fault:
      return False
    goal = FollowJointTrajectoryGoal(
      trajectory = trajectory,
      goal_time_tolerance = goal_time_tolerance
    )
    self._follow_action_client.send_goal(goal, done_cb, active_cb, feedback_cb)

  def stop_arm_if_fault(self, _feedback):
    """
    stops arm if arm fault exists during feedback callback
    """
    if self.arm_motion_continues_in_fault is False and self.arm_fault:
      self.stop()

  def success(self):
    return not self.arm_fault

  def stop(self):
    """
    Stops the execution of the last trajectory submitted for executoin
    """
    action_state = self.get_state()
    if action_state == GoalStatus.ACTIVE:
      self._follow_action_client.cancel_goal()

  def wait(self, timeout=0):
    """
    Blocks until the execution of the current trajectory comes to an end
    :type timeout: int
    """
    self._follow_action_client.wait_for_result(timeout=rospy.Duration(timeout))

  def result(self):
    """
    Gets the result of the last goal
    """
    return self._follow_action_client.get_result()

  def was_preempted(self):
    return self._follow_action_client.get_state() == GoalStatus.PREEMPTED
