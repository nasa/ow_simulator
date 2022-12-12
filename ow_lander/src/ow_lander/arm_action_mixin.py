# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import actionlib
import moveit_commander

from ow_lander.trajectory_executor import ArmTrajectoryExecutor
from ow_lander.trajectory_planner import ArmTrajectoryPlanner
from ow_lander.subscribers import LinkPositionSubscriber

from abc import ABC, abstractmethod

class ArmActionMixin:
  """Enables an action server to control the OceanWATERS arm. This or one of its
  children class must be placed first in the inheritance statement.
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
    self._arm_tip_monitor = LinkPositionSubscriber('lander::l_scoop_tip')
    self._start_server()

  def publish_action_feedback(self):
    """Publishes the action's feedback. Most arm actions publish the scoop tip
    position under the name "current" in their feedback, so that is what this
    method does by default, but it can be overridden by the child class.
    """
    self._publish_feedback(current=self._arm_tip_monitor.get_link_position())

class ArmToGroupStateMixin(ArmActionMixin, ABC):
  """A specialization of an arm action mixin that moves the arm directly to
  a group state, which is a series of joint angles."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  """The named group state which the arm will move directly to when this
  particular type of action is executed. Must be set by child class!
  """
  @property
  @abstractmethod
  def target_group_state(self):
    pass

  def execute_action(self, _goal):
    try:
      self._arm.checkout_arm(self.name)
      plan = self._planner.plan_arm_to_target(self.target_group_state)
      self._arm.execute_arm_trajectory(plan)
    except RuntimeError as err:
      self._set_aborted(str(err),
        final=self._arm_tip_monitor.get_link_position())
    else:
      self._set_succeeded("Arm trajectory succeeded",
        final=self._arm_tip_monitor.get_link_position())
    finally:
      self._arm.checkin_arm()

class ArmInterface:
  """Implements an ownership layer over trajectory execution and a stop method.
  Ownership is claimed/relinquished via the check out/in methods.
  """

  # string that identifies what facility is using the arm
  # if None, arm is not in use
  _in_use_by = None
  # true if arm is checked out and stop_arm was called
  # set to false when arm is checked in
  _stopped = False

  @classmethod
  def checkout_arm(cls, owner):
    if cls._in_use_by:
      raise RuntimeError(
        f"Arm is already checked out by the {cls._in_use_by} action server")
    cls._in_use_by = owner

  @classmethod
  def checkin_arm(cls):
    cls._in_use_by = None
    cls._stopped = False

  @classmethod
  def stop_arm(cls):
    if cls._in_use_by:
      cls._stopped = True
    return cls._stopped

  @classmethod
  def _assert_arm_is_checked_out(cls):
    if not cls._in_use_by:
      raise RuntimeError("Arm action did not check-out the arm before " \
                         "attempting to execute a trajectory.")

  def __init__(self):
    # initialize/reference trajectory execution singleton
    self._executor = ArmTrajectoryExecutor()

  def switch_to_grinder_controller(self):
    ArmInterface._assert_arm_is_checked_out()
    if self._executor.active_controller == 'grinder_controller':
      return
    if not self._executor.switch_controllers('grinder_controller',
                                             'arm_controller'):
      raise RuntimeError("Failed to grinder_controller")

  def switch_to_arm_controller(self):
    ArmInterface._assert_arm_is_checked_out()
    if self._executor.active_controller == 'arm_controller':
      return
    if not self._executor.switch_controllers('arm_controller',
                                             'grinder_controller'):
      raise RuntimeError("Failed to switch to arm_controller")

  def execute_arm_trajectory(self, plan, feedback_publish_cb=None):
    """Executes the provided plan and awaits its completions
    plan -- An instance of moveit_msgs.msg.RobotTrajectory that describes the
            arm trajectory to be executed. Can be None, in which case planning
            is assumed to have failed.
    feedback_publish_cb -- A function called at 100 Hz during execution of a
                           trajectory. Exists to publish the action's feedback
                           message. Handles no arguments.
    returns True if plan was executed successfully and without preempt.
    """

    ArmInterface._assert_arm_is_checked_out()

    if plan is None:
      raise RuntimeError("Trajectory planning failed")

    if ArmInterface._stopped:
      raise RuntimeError("Stop was called; trajectory will not be executed")

    self._executor.execute(plan.joint_trajectory)

    # publish feedback while waiting for trajectory execution completion
    # NOTE : Looping for a timeout like this is prone to errors and results in a
    #        large discontinuity between the final "current" value (in feedback)
    #        and the "final" value (in result). This is at least partially
    #        responsible for final positions varying far more than the eye can
    #        see in the simulation because previously the "final" value was
    #        assigned to whatever the most recent "current" value was, even if
    #        timeout caused that "current" value to be grabbed milliseconds
    #        before the actual completion of the action.
    #        Ideally this would loop so long as _executor is active, or in other
    #        words, so long as the active follow_joint_trajectory action client
    #        in _executor returns a get_state() of 1 (ACTIVE). However, this is
    #        bugged and an aborted state is commonly returned by this method in
    #        the middle of a trajectory. See OW-1090 for more details.
    FEEDBACK_RATE = 100 # hertz
    rate = rospy.Rate(FEEDBACK_RATE) # hertz
    timeout = plan.joint_trajectory.points[-1].time_from_start \
              - plan.joint_trajectory.points[0].time_from_start
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < timeout.secs:
      if ArmInterface._stopped:
        self._executor.cease_execution()
        raise RuntimeError("Stop was called; trajectory execution ceased")
      if feedback_publish_cb:
        feedback_publish_cb()
      rate.sleep()

    # wait for action to complete in case it takes longer than the timeout
    self._executor.wait()
