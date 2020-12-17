#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Point
from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController
from ow_lander.srv import *
from ow_lander.msg import *
from actionlib_msgs.msg import GoalStatus

import activity_full_digging_traj
import activity_guarded_move
import activity_deliver_sample
import activity_grind

from trajectory_async_execution import TrajectoryAsyncExecuter
from ground_detection import GroundDetector


class PathPlanningCommander(object):

  def __init__(self):
    super(PathPlanningCommander, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    self.arm_move_group = moveit_commander.MoveGroupCommander("arm")
    self.limbs_move_group = moveit_commander.MoveGroupCommander("limbs")
    self.grinder_move_group = moveit_commander.MoveGroupCommander("grinder")
    self.trajectory_async_executer = TrajectoryAsyncExecuter()

  # === SERVICE ACTIVITIES - Stow =============================
  def handle_stow(self, req):
    """
    :type req: class 'ow_lander.srv._Stow.StowRequest'
    """
    print("Stow arm activity started")
    goal = self.arm_move_group.get_named_target_values("arm_stowed")
    self.arm_move_group.go(goal, wait=True)
    self.arm_move_group.stop()
    print("Stow arm activity completed")
    return True, "Done"

  # === SERVICE ACTIVITIES - Unstow =============================
  def handle_unstow(self, req):
    """
    :type req: class 'ow_lander.srv._Unstow.UnstowRequest'
    """
    print("Unstow arm activity started")
    goal = self.arm_move_group.get_named_target_values("arm_unstowed")
    self.arm_move_group.go(goal, wait=True)
    self.arm_move_group.stop()
    print("Unstow arm activity completed")
    return True, "Done"

  # === SERVICE ACTIVITIES - deliver sample =============================
  def handle_deliver_sample(self, req):
    """
    :type req: class 'ow_lander.srv._DeliverSample.DeliverSampleRequest'
    """
    print("Deliver Sample arm activity started")
    deliver_sample_args = activity_deliver_sample.arg_parsing(req)
    success = activity_deliver_sample.deliver_sample(
        self.arm_move_group, deliver_sample_args)
    print("Deliver Sample arm activity completed")
    return success, "Done"

  # === SERVICE ACTIVITIES - Dig Linear Trench =============================
  def handle_dig_circular(self, req):
    """
    :type req: class 'ow_lander.srv._DigCircular.DigCircularRequest'
    """
    print("Dig Cicular arm activity started")
    dig_circular_args = activity_full_digging_traj.arg_parsing_circ(req)
    success = activity_full_digging_traj.dig_circular(
        self.arm_move_group,
        self.limbs_move_group,
        dig_circular_args,
        self.switch_controllers)
    print("Dig Circular arm activity completed")
    return success, "Done"

  # === SERVICE ACTIVITIES - Dig Linear Trench =============================
  def handle_dig_linear(self, req):
    """
    :type req: class 'ow_lander.srv._DigLinear.DigLinearRequest'
    """
    dig_linear_args = activity_full_digging_traj.arg_parsing_lin(req)
    success = activity_full_digging_traj.dig_linear(
        self.arm_move_group,
        dig_linear_args)
    print "Dig linear arm motion executed!"
    return success, "Done"

  def switch_controllers(self, start_controller, stop_controller):
    rospy.wait_for_service('/controller_manager/switch_controller')
    success = False
    try:
      switch_controller = rospy.ServiceProxy(
          '/controller_manager/switch_controller', SwitchController)
      success = switch_controller(
          [start_controller], [stop_controller], 2, False, 1.0)
    except rospy.ServiceException, e:
      print("switch_controllers error: %s" % e)
    finally:
      return success

  # === SERVICE ACTIVITIES - Grind =============================
  def handle_grind(self, req):
    """
    :type req: class 'ow_lander.srv._Grind.GrindRequest'
    """
    print("Grinde arm activity started")
    grind_args = activity_grind.arg_parsing(req)
    success = self.switch_controllers('grinder_controller', 'arm_controller')
    if not success:
      return False, "Failed"
    success = activity_grind.grind(
        self.grinder_move_group,
        grind_args)
    self.switch_controllers('arm_controller', 'grinder_controller')
    print("Grinde arm activity completed")
    return success, "Done"

  def handle_guarded_move_done(self, state, result):
    ground_detected = state == GoalStatus.PREEMPTED
    ground_position = self.ground_detector.ground_position if ground_detected else Point()
    rospy.loginfo("Ground Detected ? {}".format(ground_detected))
    self.guarded_move_pub.publish(ground_detected, 'base_link', ground_position)

  def handle_guarded_move_feedback(self, feedback):
    if self.ground_detector.detect():
      self.trajectory_async_executer.stop()

  # === SERVICE ACTIVITIES - guarded move =============================
  def handle_guarded_move(self, req):
    """
    :type req: class 'ow_lander.srv._GuardedMove.GuardedMoveRequest'
    """
    print("Guarded Move arm activity started")
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
                                           active_cb=None,
                                           feedback_cb=self.handle_guarded_move_feedback)
    # TODO: the async execute returns immediately after submitting a trajectory
    # to maintaing current behavior on melodic-devel we can insert a wait like:
    # self.trajectory_async_executer.wait(time_to_start of last point)
    print("Guarded Move arm activity completed")
    return success, "Done"

  def run(self):
    rospy.init_node('path_planning_commander', anonymous=True)
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
        '/guarded_move_result', GuardedMoveResult, queue_size=10)
    self.guarded_move_srv = rospy.Service(
        'arm/guarded_move', GuardedMove, self.handle_guarded_move)
    print("path_planning_commander has started!")
    rospy.spin()


if __name__ == '__main__':
  ppc = PathPlanningCommander()
  ppc.run()
