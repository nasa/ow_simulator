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

class ArmActionMixin:
  """Enables an action server to control the OceanWATERS arm. Must be placed
  first in the inheritance statement.
  e.g.
  class GenericArmAction(ArmActionMixin, ActionServerBase):
    ...
  """

  # true if arm is in use by an arm action
  arm_in_use = False
  # true if _stop_arm was called and arm was checked out
  # reverts to false when arm is checked in
  stopped = False

  @classmethod
  def _stop_arm(cls):
    if cls.arm_in_use:
      cls.stopped = True
    return cls.stopped

  @classmethod
  def _checkout_arm(cls):
    if cls.arm_in_use:
      raise RuntimeError("Arm is already checked out by another action server")
    cls.arm_in_use = True

  @classmethod
  def _checkin_arm(cls):
    cls.arm_in_use = False
    cls.stopped = False

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    # initialize moveit interface for arm control
    moveit_commander.roscpp_initialize(sys.argv)

    # initialize/reference trajectory planner singleton
    self._planner = ArmTrajectoryPlanner()
    # initialize/reference trajectory execution singleton
    self._executor = ArmTrajectoryExecutor()
    # initialize interface for querying scoop tip position
    self._arm_tip = LinkPositionSubscriber('lander::l_scoop_tip')
    # initialize stop flag
    # NOTE: True when the stop action is called. Always set to false upon action
    #       completion (success or abort). Enables stopping an action during
    #       the planning phase.

  def _execution_feedback_cb(self, _feedback):
    """Called during the trajectory execution. Does nothing by default, but
    optionally can be override by the child class.
    _feedback -- An instance of control_msgs.msg.FollowJointTrajectoryFeedback
    """
    pass

  def _execution_active_cb(self):
    """Called when the trajectory execution begins. Does nothing by default, but
    optionally can be overridden by the child class.
    """
    pass

  def _execution_done_cb(self, _state, _result):
    """Called when the trajectory execution completes. Does nothing by default,
    but optionally can be overridden by the child class.
    _state  -- An instance of actionlib_msgs.GoalStatus that provides the final
               state of the action.
    _result -- An instance of control_msgs.msg.FollowJointTrajectoryResult
    """
    pass

  def _publish_action_feedback(self):
    """Publishes the action's feedback. Most arm actions publish the scoop tip
    position under the name "current" in their feedback, so that is what this
    method does by default, but it can be overridden by the child class.

    This method is called at a rate of 100 Hertz while the trajectory is
    executed.
    """
    self._publish_feedback(current=self._get_arm_tip_position())

  def _get_arm_tip_position(self):
    return self._arm_tip.get_link_position()

  def _switch_to_grinder_controller(self):
    if self._executor.active_controller == 'grinder_controller':
      return
    if not self._executor.switch_controllers('grinder_controller',
                                             'arm_controller'):
      raise RuntimeError("Failed to grinder_controller")

  def _switch_to_arm_controller(self):
    if self._executor.active_controller == 'arm_controller':
      return
    if not self._executor.switch_controllers('arm_controller',
                                             'grinder_controller'):
      raise RuntimeError("Failed to switch to arm_controller")

  def _execute_arm_trajectory(self, plan):
    """Executes the provided plan and awaits its completions
    plan - An instance of moveit_msgs.msg.RobotTrajectory that describes the
           arm trajectory to be executed. Can be None, in which case planning
           is assumed to have failed.
    returns True if plan was executed successfully and without preempt.
    """

    if plan is None:
      raise RuntimeError("Trajectory planning failed")

    if ArmActionMixin.stopped:
      raise RuntimeError("Stop was called; trajectory will not be executed")

    # DEACTIVATED: still investigating how best to incorporate the stop action
    # if _server_stop.stopped:
    #   self._set_aborted(self.result_type(), "Stop server is stopped")
    #   return

    self._executor.execute(
      plan.joint_trajectory,
      active_cb=self._execution_active_cb,
      feedback_cb=self._execution_feedback_cb,
      done_cb=self._execution_done_cb
    )

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
    #        in _executor returns a get_state() of 1. This however is bugged,
    #        and returning an aborted state is common enough that this cannot
    #        be done. See OW-1090 for more details.
    FEEDBACK_RATE = 100 # hertz
    rate = rospy.Rate(FEEDBACK_RATE) # hertz
    timeout = plan.joint_trajectory.points[-1].time_from_start \
              - plan.joint_trajectory.points[0].time_from_start
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < timeout.secs:
      if ArmActionMixin.stopped:
        self._executor.cease_execution()
        raise RuntimeError("Stop was called; trajectory execution ceased")
      self._publish_action_feedback()
      rate.sleep()

    # wait for action to complete in case it takes longer than the timeout
    self._executor.wait()
