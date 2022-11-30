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
    self._arm_trajectory_planner = ArmTrajectoryPlanner()
    # initialize/reference trajectory execution singleton
    self._arm_trajectory_executor = ArmTrajectoryExecutor()
    # # initialize/reference stop singleton
    # self._arm_stop_server =
    # initialize interface for querying scoop tip position
    self._arm_tip = LinkPositionSubscriber('lander::l_scoop_tip')

  def get_arm_tip_position(self):
    return self._arm_tip.get_link_position()

  def feedback_callback(self, _feedback):
    pass

  def active_callback(self):
    pass

  def done_callback(self, _state, _result):
    pass

  def execute_arm_trajectory(self, plan,
      done_cb = None, active_cb = None, feedback_cb = None):

    if plan is None:
      self._set_aborted(self.result_type(), "Plan was aborted")

    # if _server_stop.stopped:
    #   self._set_aborted(self.result_type(), "Stop server is stopped")
    #   return

    self._arm_trajectory_executor.execute(
      plan.joint_trajectory,
      active_cb=self.active_callback,
      feedback_cb=self.feedback_callback,
      done_cb=self.done_callback
    )

    # publish feedback while waiting for trajectory execution completion
    FEEDBACK_RATE = 100 # hertz
    rate = rospy.Rate(FEEDBACK_RATE) # hertz
    timeout = plan.joint_trajectory.points[-1].time_from_start \
              - plan.joint_trajectory.points[0].time_from_start
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < timeout.secs: #and not _server_stop.stopped:
      self._publish_feedback(
        self.feedback_type(current=self.get_arm_tip_position())
      )
      rate.sleep()

    self._arm_trajectory_executor.wait()

    success = self._arm_trajectory_executor.success() \
              and not self._arm_trajectory_executor.was_preempted()

    if success:
      self._set_succeeded(
        self.result_type(final=self.get_arm_tip_position())
      )
    else:
      self._set_aborted(self.result_type(final=self.get_arm_tip_position()))
