#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import ow_lander.msg
import sys
import action_guarded_move
from LanderInterface import MoveItInterface
from LanderInterface import LinkStateSubscriber
from trajectory_async_execution import TrajectoryAsyncExecuter
from moveit_msgs.msg import RobotTrajectory
from ground_detection import GroundDetector
from ow_lander.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, WrenchStamped


class GuardedMoveActionServer(object):
    
    def __init__(self,name):
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, 
                                     ow_lander.msg.GuardedMoveAction, 
                                     execute_cb=self.on_guarded_move_action, 
                                     auto_start = False)
        self._server.start()
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.GuardedMoveFeedback()
        self._result = ow_lander.msg.GuardedMoveResult()
        self._current_link_state = LinkStateSubscriber()
        self._interface = MoveItInterface()
        self._timeout = 0.0
        self.trajectory_async_executer = TrajectoryAsyncExecuter()
        self.trajectory_async_executer.connect("arm_controller")
        self.guarded_move_traj = RobotTrajectory()
        self.ground_detector = GroundDetector()
        self.pos = Point()
        self.guarded_move_pub = rospy.Publisher(
        '/guarded_move_result', GuardedMoveFinalResult, queue_size=10)
        
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
          if (self.ground_detector.ground_position.z) > 0.1 :
            self.ground_detector.reset()
          else:
            self.trajectory_async_executer.stop()    
        
    
    def _update_feedback(self):
 
        self._ls =  self._current_link_state._link_value
        self._fdbk.current.x = self._ls.x
        self._fdbk.current.y = self._ls.y
        self._fdbk.current.z = self._ls.z
        self._server.publish_feedback(self._fdbk)

        
        
    def _update_motion(self, goal):
        print("Guarded move activity started")
        self.guarded_move_traj = action_guarded_move.guarded_move_plan(self._interface.move_arm,
                                              self._interface.robot, 
                                              self._interface.moveit_fk, goal)
        n_points = len(self.guarded_move_traj.joint_trajectory.points)
        start_time =   self.guarded_move_traj.joint_trajectory.points[0].time_from_start
        end_time = self.guarded_move_traj.joint_trajectory.points[n_points-1].time_from_start
        self._timeout = end_time -start_time
        
        
        
    def on_guarded_move_action(self,goal):
        plan = self._update_motion(goal)
        success = False


        # detection
        self.ground_detector.reset()

        self.trajectory_async_executer.execute(self.guarded_move_traj.joint_trajectory,
                                           done_cb=self.handle_guarded_move_done,
                                           active_cb=None,
                                           feedback_cb=self.handle_guarded_move_feedback)

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)

        while ((now_from_start(start_time) < self._timeout)):

           self._update_feedback()
           
        success = self.trajectory_async_executer.wait()
        
            
        if success:
            self._result.final.x = self.ground_detector.ground_position.x
            self._result.final.y = self.ground_detector.ground_position.y
            self._result.final.z = self.ground_detector.ground_position.z
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._server.set_succeeded(self._result)
    
if __name__ == '__main__':
    rospy.init_node('Guarded_move')
    server = GuardedMoveActionServer(rospy.get_name())
    rospy.spin()
        
  
