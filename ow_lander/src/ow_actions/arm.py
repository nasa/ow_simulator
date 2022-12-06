# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import actionlib
import moveit_commander

from ow_actions.trajectory_executor import ArmTrajectoryExecutor
from ow_actions.trajectory_planner import ArmTrajectoryPlanner
from ow_actions.subscribers import LinkPositionSubscriber

class ArmActionMixin:

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
    self._stopped = False

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

  def _switch_to_grind_controller(self):
    return self._executor.switch_controllers(
      'grinder_controller', 'arm_controller')

  def _switch_to_arm_controller(self):
    return self._executor.switch_controllers(
      'arm_controller', 'grinder_controller')

  def _stop_arm_action(self):
    return self._executor.stop()

  def _execute_arm_trajectory(self, plan):
    """Executes the provided plan and awaits its completions
    plan - An instance of moveit_msgs.msg.RobotTrajectory that describes the
           arm trajectory to be executed. Can be None, in which case planning
           is assumed to have failed.
    returns True if plan was executed successfully and without preempt.
    """

    if plan is None:
      self._set_aborted("Trajectory planning failed")

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
    # TODO: verify that timeout is the appropriate time to loop for
    FEEDBACK_RATE = 100 # hertz
    rate = rospy.Rate(FEEDBACK_RATE) # hertz
    timeout = plan.joint_trajectory.points[-1].time_from_start \
              - plan.joint_trajectory.points[0].time_from_start
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < timeout.secs \
        and not self._executor.is_stopped():
      self._publish_action_feedback()
      rate.sleep()

    self._executor.wait()

    self._executor.reset_stopped_state()

    return self._executor.success() and not self._executor.is_preempted()

