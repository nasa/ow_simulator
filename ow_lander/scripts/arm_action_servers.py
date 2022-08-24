#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
from ow_lander.msg import *
from LanderInterface import MoveItInterface
from LanderInterface import LinkStateSubscriber
from LanderInterface import ArmJointStateSubscriber
import numpy as np
from trajectory_async_execution import TrajectoryAsyncExecuter
from action_trajectories import ActionTrajectories
from moveit_msgs.msg import RobotTrajectory
from controller_manager_msgs.srv import SwitchController
from ground_detection import GroundDetector
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from math import pi


class UnstowActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.UnstowFeedback()
        self._result = ow_lander.msg.UnstowResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.UnstowAction,
                                                    execute_cb=self.on_unstow_action,
                                                    auto_start=False)
        self._server.start()

    def _update_feedback(self):

        self._ls = self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self):

        rospy.loginfo("Unstow arm activity started")
        goal = self._interface.move_arm.get_current_pose().pose
        goal = self._interface.move_arm.get_named_target_values("arm_unstowed")
        self._interface.move_arm.set_joint_value_target(goal)
        _, plan, _, _ = self._interface.move_arm.plan()
        if len(plan.joint_trajectory.points) < 1:
            return
        else:
            n_points = len(plan.joint_trajectory.points)
            start_time = plan.joint_trajectory.points[0].time_from_start
            end_time = plan.joint_trajectory.points[n_points-1].time_from_start
            self._timeout = end_time - start_time
            return plan

    def on_unstow_action(self, goal):
        server_stop.reset()
        plan = self._update_motion()
        if plan is None:
            self._server.set_aborted(self._result)
            return
        success = False
        if server_stop.stopped is False:
            trajectory_async_executer.execute(plan.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            rospy.loginfo('Unstow was stopped.')
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final.x = self._fdbk.current.x
            self._result.final.y = self._fdbk.current.y
            self._result.final.z = self._fdbk.current.z
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


class StowActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.StowFeedback()
        self._result = ow_lander.msg.StowResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.StowAction,
                                                    execute_cb=self.on_stow_action,
                                                    auto_start=False)
        self._server.start()

    def _update_feedback(self):

        self._ls = self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self):

        rospy.loginfo("Stow arm activity started")
        goal = self._interface.move_arm.get_current_pose().pose
        goal = self._interface.move_arm.get_named_target_values("arm_stowed")
        self._interface.move_arm.set_joint_value_target(goal)

        _, plan, _, _ = self._interface.move_arm.plan()
        if len(plan.joint_trajectory.points) < 1:
            return
        else:
            n_points = len(plan.joint_trajectory.points)
            start_time = plan.joint_trajectory.points[0].time_from_start
            end_time = plan.joint_trajectory.points[n_points-1].time_from_start
            self._timeout = end_time - start_time
            return plan

    def on_stow_action(self, goal):
        server_stop.reset()
        plan = self._update_motion()
        if plan is None:
            self._server.set_aborted(self._result)
            return
        success = False
        if server_stop.stopped is False:
            trajectory_async_executer.execute(plan.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final.x = self._fdbk.current.x
            self._result.final.y = self._fdbk.current.y
            self._result.final.z = self._fdbk.current.z
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


class StopActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.StopFeedback()
        self._result = ow_lander.msg.StopResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self.stopped = False
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.StopAction,
                                                    execute_cb=self.on_stop_action,
                                                    auto_start=False)
        self._server.start()

    def reset(self):
        self.stopped = False

    def get_state(self):
        return self.stopped

    def on_stop_action(self, goal):

        self._ls = self._current_link_state._link_value
        self._result.final.x = self._ls.x
        self._result.final.y = self._ls.y
        self._result.final.z = self._ls.z

        self.stopped = True
        trajectory_async_executer.stop()

        rospy.loginfo('%s: Succeeded' % self._action_name)
        rospy.loginfo('Current Arm Action was stopped.')
        self._server.set_succeeded(self._result)


class GrindActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.GrindFeedback()
        self._result = ow_lander.msg.GrindResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self.current_traj = RobotTrajectory()
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.GrindAction,
                                                    execute_cb=self.on_Grind_action,
                                                    auto_start=False)
        self._server.start()

    def _update_feedback(self):

        self._ls = self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

    def switch_controllers(self, start_controller, stop_controller):
        rospy.wait_for_service('/controller_manager/switch_controller')
        success = False
        try:
            switch_controller = rospy.ServiceProxy(
                '/controller_manager/switch_controller', SwitchController)
            success = switch_controller(
                [start_controller], [stop_controller], 2, False, 1.0)
        except rospy.ServiceException as e:
            rospy.loginfo("switch_controllers error: %s" % e)
        finally:
            # This sleep is a workaround for "start point deviates from current robot
            #  state" error on dig_circular trajectory execution.
            rospy.sleep(0.2)
            return success

    def _update_motion(self, goal):
        rospy.loginfo("Grind activity started")
        self.current_traj = action_trajectories.grind(self._interface.move_grinder,
                                                      self._interface.robot,
                                                      self._interface.moveit_fk, goal, server_stop)
        if self.current_traj == False:
            return
        else:
            n_points = len(self.current_traj.joint_trajectory.points)
            start_time = self.current_traj.joint_trajectory.points[0].time_from_start
            end_time = self.current_traj.joint_trajectory.points[n_points -
                                                                 1].time_from_start
            self._timeout = (end_time - start_time)

    def switch_to_grind_controller(self):
        success = False
        switch_success = self.switch_controllers(
            'grinder_controller', 'arm_controller')
        if not switch_success:
            return False, "Failed switching controllers"
        # connect grinder controller to the trajectory executer
        trajectory_async_executer.connect("grinder_controller")

    def switch_to_arm_controller(self):
        switch_success = self.switch_controllers(
            'arm_controller', 'grinder_controller')
        if not switch_success:
            return False, "Failed Switching Controllers"
            # switch controller after completing grind operation
        trajectory_async_executer.connect("arm_controller")

    def on_Grind_action(self, goal):
        server_stop.reset()
        self._update_motion(goal)
        if self.current_traj == False:
            self._server.set_aborted(self._result)
            return
        success = False

        self.switch_to_grind_controller()

        if server_stop.stopped is False:
            trajectory_async_executer.execute(self.current_traj.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            self.switch_to_arm_controller()
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final.x = self._fdbk.current.x
            self._result.final.y = self._fdbk.current.y
            self._result.final.z = self._fdbk.current.z

            self.switch_to_arm_controller()

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)
            self.switch_to_arm_controller()


class GuardedMoveActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.GuardedMoveFeedback()
        self._result = ow_lander.msg.GuardedMoveResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self._estimated_plan_fraction_completed = 0.0
        # ratio between guarded pre-guarded move trajectory and the whole trajectory
        self._guarded_move_plan_ratio = 0.0
        self.guarded_move_traj = RobotTrajectory()
        self.ground_detector = GroundDetector()
        self.pos = Point()
        self.guarded_move_pub = rospy.Publisher(
            '/guarded_move_result', GuardedMoveFinalResult, queue_size=10)
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.GuardedMoveAction,
                                                    execute_cb=self.on_guarded_move_action,
                                                    auto_start=False)
        self._server.start()

    def handle_guarded_move_done(self, state, result):
        """
        :type state: int
        :type result: FollowJointTrajectoryResult
        """
        ground_detected = state == GoalStatus.PREEMPTED and server_stop.stopped is False
        ground_position = self.ground_detector.ground_position if ground_detected else Point()
        rospy.loginfo("Ground Detected ? {}".format(ground_detected))
        self.guarded_move_pub.publish(
            ground_detected, 'base_link', ground_position)

    def handle_guarded_move_feedback(self, feedback):
        """
        :type feedback: FollowJointTrajectoryFeedback
        """
        trajectory_async_executer.stop_arm_if_fault(feedback)
        # added to compensate for slower than arm movement than planned
        execution_time_tollerance = 0.1

        '''
        If the force torque sensor detects a force greater than the threshold it reports as ground detected.
        We activate the ground detection only during the last phase of the guarded move trajectory, 
        reseting it otherwise
        '''

        if self.ground_detector.detect():
            if (self._estimated_plan_fraction_completed < self._guarded_move_plan_ratio
                    + execution_time_tollerance):
                self.ground_detector.reset()
            else:
                trajectory_async_executer.stop()

    def _update_feedback(self):

        self._ls = self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self, goal):
        rospy.loginfo("Guarded move activity started")
        self.guarded_move_traj, self._guarded_move_plan_ratio = action_trajectories.guarded_move_plan(
            self._interface.move_arm,
            self._interface.robot,
            self._interface.moveit_fk, goal, server_stop)

        if self.guarded_move_traj == False:
            return
        else:
            n_points = len(self.guarded_move_traj.joint_trajectory.points)
            start_time = self.guarded_move_traj.joint_trajectory.points[0].time_from_start
            end_time = self.guarded_move_traj.joint_trajectory.points[n_points-1].time_from_start
            self._timeout = end_time - start_time

    def on_guarded_move_action(self, goal):
        server_stop.reset()
        self._update_motion(goal)
        if self.guarded_move_traj == False:
            self._server.set_aborted(self._result)
            return
        success = False

        # detection
        self.ground_detector.reset()

        if server_stop.stopped is False:
            trajectory_async_executer.execute(self.guarded_move_traj.joint_trajectory,
                                              done_cb=self.handle_guarded_move_done,
                                              active_cb=None,
                                              feedback_cb=self.handle_guarded_move_feedback)
        else:
            self._server.set_aborted(self._result)
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()
            self._estimated_plan_fraction_completed = now_from_start(
                start_time)/self._timeout

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() == GoalStatus.PREEMPTED and server_stop.stopped is False

        if success:
            self._result.final.x = self.ground_detector.ground_position.x
            self._result.final.y = self.ground_detector.ground_position.y
            self._result.final.z = self.ground_detector.ground_position.z
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


class DigCircularActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.DigCircularFeedback()
        self._result = ow_lander.msg.DigCircularResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self.current_traj = RobotTrajectory()
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.DigCircularAction,
                                                    execute_cb=self.on_DigCircular_action,
                                                    auto_start=False)
        self._server.start()

    def switch_controllers(self, start_controller, stop_controller):
        rospy.wait_for_service('/controller_manager/switch_controller')
        success = False
        try:
            switch_controller = rospy.ServiceProxy(
                '/controller_manager/switch_controller', SwitchController)
            success = switch_controller(
                [start_controller], [stop_controller], 2, False, 1.0)
        except rospy.ServiceException as e:
            rospy.loginfo("switch_controllers error: %s" % e)
        finally:
            # This sleep is a workaround for "start point deviates from current robot
            #  state" error on dig_circular trajectory execution.
            rospy.sleep(0.2)
            return success

    def _update_feedback(self):

        self._ls = self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self, goal):
        rospy.loginfo("DigCircular activity started")
        self.current_traj = None
        self.current_traj = action_trajectories.dig_circular(self._interface.move_arm,
                                                             self._interface.move_limbs,
                                                             self._interface.robot, self._interface.moveit_fk, goal, server_stop)
        if self.current_traj == False:
            return
        else:
            n_points = len(self.current_traj.joint_trajectory.points)
            start_time = self.current_traj.joint_trajectory.points[0].time_from_start
            end_time = self.current_traj.joint_trajectory.points[n_points -
                                                                 1].time_from_start
            self._timeout = (end_time - start_time)

    def on_DigCircular_action(self, goal):
        server_stop.reset()
        self._update_motion(goal)
        if self.current_traj == False:
            self._server.set_aborted(self._result)
            return
        success = False

        if server_stop.stopped is False:
            trajectory_async_executer.execute(self.current_traj.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout)) and server_stop.stopped is False:

            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final.x = self._fdbk.current.x
            self._result.final.y = self._fdbk.current.y
            self._result.final.z = self._fdbk.current.z
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


class DigLinearActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.DigLinearFeedback()
        self._result = ow_lander.msg.DigLinearResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self.current_traj = RobotTrajectory()
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.DigLinearAction,
                                                    execute_cb=self.on_DigLinear_action,
                                                    auto_start=False)
        self._server.start()

    def _update_feedback(self):

        self._ls = self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self, goal):

        rospy.loginfo("DigLinear activity started")
        self.current_traj = action_trajectories.dig_linear(self._interface.move_arm,
                                                           self._interface.robot,
                                                           self._interface.moveit_fk, goal, server_stop)
        if self.current_traj == False:
            return
        else:
            n_points = len(self.current_traj.joint_trajectory.points)
            start_time = self.current_traj.joint_trajectory.points[0].time_from_start
            end_time = self.current_traj.joint_trajectory.points[n_points -
                                                                 1].time_from_start
            self._timeout = (end_time - start_time)

    def on_DigLinear_action(self, goal):
        server_stop.reset()
        self._update_motion(goal)
        if self.current_traj == False:
            self._server.set_aborted(self._result)
            return

        success = False

        if server_stop.stopped is False:
            trajectory_async_executer.execute(self.current_traj.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final.x = self._fdbk.current.x
            self._result.final.y = self._fdbk.current.y
            self._result.final.z = self._fdbk.current.z
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


class DiscardActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.DiscardFeedback()
        self._result = ow_lander.msg.DiscardResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self.discard_sample_traj = RobotTrajectory()
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.DiscardAction,
                                                    execute_cb=self.on_discard_action,
                                                    auto_start=False)
        self._server.start()

    def _update_feedback(self):
        self._ls = self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self, goal):
        rospy.loginfo("Discard sample activity started")
        self.discard_sample_traj = action_trajectories.discard_sample(self._interface.move_arm,
                                                                      self._interface.robot,
                                                                      self._interface.moveit_fk, goal, server_stop)
        if self.discard_sample_traj == False:
            return
        else:
            n_points = len(self.discard_sample_traj.joint_trajectory.points)
            start_time = self.discard_sample_traj.joint_trajectory.points[0].time_from_start
            end_time = self.discard_sample_traj.joint_trajectory.points[n_points-1].time_from_start
            self._timeout = end_time - start_time

    def on_discard_action(self, goal):
        server_stop.reset()
        self._update_motion(goal)
        if self.discard_sample_traj == False:
            self._server.set_aborted(self._result)
            return
        success = False

        if server_stop.stopped is False:
            trajectory_async_executer.execute(self.discard_sample_traj.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final.x = self._fdbk.current.x
            self._result.final.y = self._fdbk.current.y
            self._result.final.z = self._fdbk.current.z
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


class DeliverActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.DeliverFeedback()
        self._result = ow_lander.msg.DeliverResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self.deliver_sample_traj = RobotTrajectory()
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.DeliverAction,
                                                    execute_cb=self.on_deliver_action,
                                                    auto_start=False)
        self._server.start()

    def _update_feedback(self):
        self._ls = self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self):
        rospy.loginfo("Deliver sample activity started")
        self.deliver_sample_traj = action_trajectories.deliver_sample(self._interface.move_arm,
                                                                      self._interface.robot,
                                                                      self._interface.moveit_fk, server_stop)
        if self.deliver_sample_traj == False:
            return
        else:
            n_points = len(self.deliver_sample_traj.joint_trajectory.points)
            start_time = self.deliver_sample_traj.joint_trajectory.points[0].time_from_start
            end_time = self.deliver_sample_traj.joint_trajectory.points[n_points-1].time_from_start
            self._timeout = end_time - start_time
    # executive call back of simple action server requires a dummy goal

    def on_deliver_action(self, goal):
        server_stop.reset()
        self._update_motion()
        if self.deliver_sample_traj == False:
            self._server.set_aborted(self._result)
            return
        success = False

        if server_stop.stopped is False:
            trajectory_async_executer.execute(self.deliver_sample_traj.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final.x = self._fdbk.current.x
            self._result.final.y = self._fdbk.current.y
            self._result.final.z = self._fdbk.current.z
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


class ArmMoveJoint(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.ArmMoveJointFeedback()
        self._result = ow_lander.msg.ArmMoveJointResult()
        self._current_joint_state = ArmJointStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.ArmMoveJointAction,
                                                    execute_cb=self.on_ArmMoveJoint_action,
                                                    auto_start=False)
        self._server.start()

    def _update_feedback(self):
        self._js = self._current_joint_state.get_joint_angles()
        self._fdbk.angle = self._js
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self, goal):

        rospy.loginfo("Arm move joint started")
        joint_goal = self._interface.move_arm.get_current_joint_values()

        if goal.relative == False:
            joint_goal[goal.joint] = goal.angle
        else:
            joint_goal[goal.joint] = joint_goal[goal.joint] + goal.angle
        self._interface.move_arm.set_joint_value_target(joint_goal)

        _, plan, _, _ = self._interface.move_arm.plan()
        if len(plan.joint_trajectory.points) < 1:
            return
        else:
            n_points = len(plan.joint_trajectory.points)
            start_time = plan.joint_trajectory.points[0].time_from_start
            end_time = plan.joint_trajectory.points[n_points-1].time_from_start
            self._timeout = end_time - start_time
            return plan

    def on_ArmMoveJoint_action(self, goal):
        server_stop.reset()
        plan = self._update_motion(goal)
        if plan is None:
            self._server.set_aborted(self._result)
            return
        success = False
        if server_stop.stopped is False:
            trajectory_async_executer.execute(plan.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            rospy.loginfo('Arm_move Joint  was stopped.')
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final_angle = self._fdbk.angle
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


class ArmMoveJoints(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.ArmMoveJointsFeedback()
        self._result = ow_lander.msg.ArmMoveJointsResult()
        self._current_joint_state = ArmJointStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.ArmMoveJointsAction,
                                                    execute_cb=self.on_ArmMoveJoints_action,
                                                    auto_start=False)
        self._server.start()

    def _update_feedback(self):
        self._js = self._current_joint_state.get_joint_angles()
        self._fdbk.angles = self._js
        self._server.publish_feedback(self._fdbk)

    def _update_motion(self, goal):

        rospy.loginfo("Arm move joints started")
        joint_goal = self._interface.move_arm.get_current_joint_values()

        if goal.relative == False:
            joint_goal = goal.angles
        else:
            joint_goal = list(np.array(joint_goal) + np.array(goal.angles))

        self._interface.move_arm.set_joint_value_target(joint_goal)

        _, plan, _, _ = self._interface.move_arm.plan()
        if len(plan.joint_trajectory.points) < 1:
            return
        else:
            n_points = len(plan.joint_trajectory.points)
            start_time = plan.joint_trajectory.points[0].time_from_start
            end_time = plan.joint_trajectory.points[n_points-1].time_from_start
            self._timeout = end_time - start_time
            return plan

    def on_ArmMoveJoints_action(self, goal):
        server_stop.reset()
        plan = self._update_motion(goal)
        if plan is None:
            self._server.set_aborted(self._result)
            return
        success = False
        if server_stop.stopped is False:
            trajectory_async_executer.execute(plan.joint_trajectory,
                                              done_cb=None,
                                              active_cb=None,
                                              feedback_cb=trajectory_async_executer.stop_arm_if_fault)
        else:
            self._server.set_aborted(self._result)
            rospy.loginfo('Arm_move Joint  was stopped.')
            return

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout) and server_stop.stopped is False):
            self._update_feedback()

        success = trajectory_async_executer.success(
        ) and trajectory_async_executer.wait() and trajectory_async_executer.get_state() != GoalStatus.PREEMPTED

        if success:
            self._result.final_angles = self._fdbk.angles
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._server.set_aborted(self._result)


if __name__ == '__main__':
    rospy.init_node('arm_action_servers')
    # start the trajectory executer in the main file
    trajectory_async_executer = TrajectoryAsyncExecuter()
    trajectory_async_executer.connect("arm_controller")
    action_trajectories = ActionTrajectories()
    # start all the arm action servers
    server_stop = StopActionServer("Stop")
    server_unstow = UnstowActionServer("Unstow")
    server_stow = StowActionServer("Stow")
    server_grind = GrindActionServer("Grind")
    server_guarded_move = GuardedMoveActionServer("GuardedMove")
    server_dig_circular = DigCircularActionServer("DigCircular")
    server_dig_linear = DigLinearActionServer("DigLinear")
    server_deliver = DeliverActionServer("Deliver")
    server_discard = DiscardActionServer("Discard")
    server_ArmMoveJoint = ArmMoveJoint("ArmMoveJoint")
    server_ArmMoveJoints = ArmMoveJoints("ArmMoveJoints")
    rospy.spin()
