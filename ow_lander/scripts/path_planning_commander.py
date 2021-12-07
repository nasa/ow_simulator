#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Point
from controller_manager_msgs.srv import SwitchController
from actionlib_msgs.msg import GoalStatus
from ow_faults_detection.msg import ArmFaults

from ow_lander.srv import *
from ow_lander.msg import *
import activity_full_digging_traj
import activity_guarded_move
import activity_grind

from trajectory_async_execution import TrajectoryAsyncExecuter
from ground_detection import GroundDetector

class PathPlanningCommander(object):

  def __init__(self):
    super(PathPlanningCommander, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    self.arm_move_group = moveit_commander.MoveGroupCommander("arm", wait_for_servers=20.0)
    self.limbs_move_group = moveit_commander.MoveGroupCommander("limbs", wait_for_servers=20.0)
    self.grinder_move_group = moveit_commander.MoveGroupCommander("grinder", wait_for_servers=20.0)
    self.trajectory_async_executer = TrajectoryAsyncExecuter()

  # === SERVICE ACTIVITIES - Stow =============================
  def handle_stop(self, req):
    """
    :type req: class 'ow_lander.srv._Stow.StopRequest'
    """
    self.log_started("Stop")
    self.trajectory_async_executer.stop()
    return self.log_finish_and_return("Stop")
    
  # === SERVICE ACTIVITIES - Stow =============================
  def handle_stow(self, req):
    """
    :type req: class 'ow_lander.srv._Stow.StowRequest'
    """
    self.log_started("Stow")
    goal = self.arm_move_group.get_named_target_values("arm_stowed")
    self.arm_move_group.set_joint_value_target(goal)

    _, plan, _, _ = self.arm_move_group.plan()
    if len(plan.joint_trajectory.points) == 0:
      return False
    self.trajectory_async_executer.execute(plan.joint_trajectory, 
                                          feedback_cb=None)
    self.trajectory_async_executer.wait()
    return self.log_finish_and_return("Stow")


  # === SERVICE ACTIVITIES - Unstow =============================
  def handle_unstow(self, req):
    """
    :type req: class 'ow_lander.srv._Unstow.UnstowRequest'
    """
    self.log_started("Unstow")
    goal = self.arm_move_group.get_named_target_values("arm_unstowed")
    self.arm_move_group.set_joint_value_target(goal)
    _, plan, _, _ = self.arm_move_group.plan()
    if len(plan.joint_trajectory.points) == 0:
      return False
    self.trajectory_async_executer.execute(plan.joint_trajectory,
                                          feedback_cb=None)
    self.trajectory_async_executer.wait()
    return self.log_finish_and_return("Unstow")

  # === SERVICE ACTIVITIES - deliver sample =============================
  def handle_deliver_sample(self, req):
    """
    :type req: class 'ow_lander.srv._DeliverSample.DeliverSampleRequest'
    """
    self.log_started("Deliver Sample")
    success = True
    targets = [
      "arm_deliver_staging_1",
      "arm_deliver_staging_2",
      "arm_deliver_final"]
    for t in targets:
      goal = self.arm_move_group.get_named_target_values(t)
      self.arm_move_group.set_joint_value_target(goal)
      _, plan, _, _ = self.arm_move_group.plan()
      if len(plan.joint_trajectory.points) == 0:
        success = False
        break
      self.trajectory_async_executer.execute(plan.joint_trajectory)
      self.trajectory_async_executer.wait()
    return self.log_finish_and_return("Deliver Sample", success)

  # === SERVICE ACTIVITIES - Dig Linear Trench =============================
  def handle_dig_circular(self, req):
    """
    :type req: class 'ow_lander.srv._DigCircular.DigCircularRequest'
    """
    self.log_started("Dig Cicular")
    dig_circular_args = activity_full_digging_traj.arg_parsing_circ(req)
    success = activity_full_digging_traj.dig_circular(
        self.arm_move_group,
        self.limbs_move_group,
        dig_circular_args,
        self.switch_controllers)
    return self.log_finish_and_return("Dig Circular", success)

  # === SERVICE ACTIVITIES - Dig Linear Trench =============================
  def handle_dig_linear(self, req):
    """
    :type req: class 'ow_lander.srv._DigLinear.DigLinearRequest'
    """
    self.log_started("Dig Linear")
    dig_linear_args = activity_full_digging_traj.arg_parsing_lin(req)
    success = activity_full_digging_traj.dig_linear(
        self.arm_move_group,
        dig_linear_args)
    return self.log_finish_and_return("Dig Linear", success)

  def switch_controllers(self, start_controller, stop_controller):
    rospy.wait_for_service('/controller_manager/switch_controller')
    success = False
    try:
      switch_controller = rospy.ServiceProxy(
          '/controller_manager/switch_controller', SwitchController)
      success = switch_controller(
          [start_controller], [stop_controller], 2, False, 1.0)
    except rospy.ServiceException as e:
      rospy.logerror("switch_controllers error: %s" % e)
    finally:
      # This sleep is a workaround for "start point deviates from current robot
      # state" error on dig_circular trajectory execution.
      rospy.sleep(0.2)
      return success

  # === SERVICE ACTIVITIES - Grind =============================
  def handle_grind(self, req):
    """
    :type req: class 'ow_lander.srv._Grind.GrindRequest'
    """
    self.log_started("Grind")
    grind_args = activity_grind.arg_parsing(req)
    success = self.switch_controllers('grinder_controller', 'arm_controller')
    if not success:
      return False, "Failed"
    success = activity_grind.grind(
        self.grinder_move_group,
        grind_args)
    self.switch_controllers('arm_controller', 'grinder_controller')
    
    return self.log_finish_and_return("Grind", success)

  def handle_guarded_move_done(self, state, result):
    """
    :type state: int
    :type result: FollowJointTrajectoryResult
    """
    ground_detected = state == GoalStatus.PREEMPTED
    ground_position = self.ground_detector.ground_position if ground_detected else Point()
    rospy.loginfo("Ground Detected ? {}".format(ground_detected))
    self.guarded_move_pub.publish(
        ground_detected, 'base_link', ground_position)

  def handle_guarded_move_feedback(self, feedback):
    """
    :type feedback: FollowJointTrajectoryFeedback
    """
    if self.ground_detector.detect():
      self.trajectory_async_executer.stop()

  # === SERVICE ACTIVITIES - guarded move =============================
  def handle_guarded_move(self, req):
    """
    :type req: class 'ow_lander.srv._GuardedMove.GuardedMoveRequest'
    """
    self.log_started("Guarded Move")
    guarded_move_args = activity_guarded_move.arg_parsing(req)
    success = activity_guarded_move.pre_guarded_move(
        self.arm_move_group, guarded_move_args)
    if not success:
      return False, "pre_guarded_move failed"
    plan = activity_guarded_move.guarded_move_plan(
        self.arm_move_group, guarded_move_args)
    if len(plan.joint_trajectory.points) == 0:
      return False, "guarded_move_plan failed"
    self.ground_detector.reset()
    self.trajectory_async_executer.execute(plan.joint_trajectory,
                                          done_cb=self.handle_guarded_move_done,
                                          feedback_cb=self.handle_guarded_move_feedback)
    # To preserve the previous behaviour we are adding a blocking call till the
    # execution of the trajectory is completed
    self.trajectory_async_executer.wait()

    return self.log_finish_and_return("Guarded Move", success)

  def run(self):
    rospy.init_node('path_planning_commander', anonymous=True)
    self.arm_faults_sub = rospy.Subscriber(
        '/faults/arm_faults_status', ArmFaults, self.handle_arm_faults)
    self.stop_srv = rospy.Service(
        'arm/stop', Stop, self.handle_stop)
    self.stow_srv = rospy.Service(
        'arm/stow', Stow, self.handle_stow)
    self.unstow_srv = rospy.Service(
        'arm/unstow', Unstow, self.handle_unstow)
    self.dig_circular_srv = rospy.Service(
        'arm/dig_circular', DigCircular, self.handle_dig_circular)
    self.dig_linear_srv = rospy.Service(
        'arm/dig_linear', DigLinear, self.handle_dig_linear)
    self.deliver_sample_srv = rospy.Service(
        'arm/deliver_sample', DeliverSample, self.handle_deliver_sample)
    self.grind_srv = rospy.Service(
        'arm/grind', Grind, self.handle_grind)
    self.ground_detector = GroundDetector()
    self.trajectory_async_executer.connect("arm_controller")
    self.guarded_move_pub = rospy.Publisher(
        '/guarded_move_result', GuardedMoveFinalResult, queue_size=10)
    self.guarded_move_srv = rospy.Service(
        'arm/guarded_move', GuardedMove, self.handle_guarded_move)
    
    rospy.loginfo("path_planning_commander has started!")

    rospy.spin()

  def log_started(self, activity_name):
    rospy.loginfo("%s arm activity started", activity_name)

  def log_finish_and_return(self, activity_name, success=True):  
    rospy.loginfo("%s arm activity completed", activity_name)
    return success, "Done"

  def handle_arm_faults(self, data):
    """
    If system fault occurs, and it is an arm failure, an arm failure flag is set for the whole class
    """
    if data.value:
      self.trajectory_async_executer.stop()

if __name__ == '__main__':
  ppc = PathPlanningCommander()
  ppc.run()
