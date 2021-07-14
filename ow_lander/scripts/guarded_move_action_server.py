#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import ow_lander.msg
import sys
import math
import action_guarded_move
from LanderInterface import MoveItInterface
from LanderInterface import LinkStateSubscriber
from trajectory_async_execution import TrajectoryAsyncExecuter
from moveit_msgs.msg import RobotTrajectory
from ground_detection import GroundDetector
from ow_lander.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, WrenchStamped
#from pilz_robot_programming import *


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
        self._plan_fraction = 0.0
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
        self.trajectory_async_executer.stop_arm_if_fault(feedback)

        if self.ground_detector.detect():
          #if (self.ground_detector.ground_position.z) > 0.1 and self._plan_fraction < 0.8 :
          #if (self.ground_detector.ground_position.z) > 0.1:    
          if (self._plan_fraction) < 0.9:       
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
        
        print (self.guarded_move_traj.joint_trajectory.points[1].velocities)
        

        #ref_state = self._interface.robot.get_current_state() # I don't know this is OK
        #plan_retimed = self._interface.move_arm.retime_trajectory(ref_state, self.guarded_move_traj, velocity_scaling_factor=0.1, acceleration_scaling_factor=0.1)
        #print (plan_retimed)
        
        #file = open("original_plan.yaml", "w")
        #file.write("%s = %s\n" %("plan_a", self.guarded_move_traj))
        #file.close()
        #file = open("modified_plan.yaml", "w")
        #file.write("%s = %s\n" %("plan_b", plan_retimed))
        #file.close()
        
        if self.guarded_move_traj == False: 
            return 
        else:
            n_points = len(self.guarded_move_traj.joint_trajectory.points)
            start_time =   self.guarded_move_traj.joint_trajectory.points[0].time_from_start
            end_time = self.guarded_move_traj.joint_trajectory.points[n_points-1].time_from_start
            self._timeout = end_time -start_time
        
        
        
    def on_guarded_move_action(self,goal):
        plan = self._update_motion(goal)
        if self.guarded_move_traj == False: 
            self._server.set_aborted(self._result)
            return 
        success = False


        # detection
        self.ground_detector.reset()

        self.trajectory_async_executer.execute(self.guarded_move_traj.joint_trajectory,
                                           done_cb=self.handle_guarded_move_done,
                                           active_cb=None,
                                           feedback_cb=self.handle_guarded_move_feedback)

        # Record start time
        start_time = rospy.get_time()
        
        def euclidean_distance (start, end): 
            return math.sqrt ((start.x - end.position.x)^2 + (start.y - end.position.y)^2+ (start.z - end.position.z)^2)
        

        def now_from_start(start):
            return rospy.Duration(secs=rospy.get_time() - start)
        #calculate end state from plan 
        #cs, start_state, goal_pose = calculate_joint_state_end_pose_from_plan_arm (self._interface.robot, self.guarded_move_traj.joint_trajectory, self._interface.move_arm, self._interface.moveit_fk)
        
        while ((now_from_start(start_time) < self._timeout)):

           self._update_feedback()
           self._plan_fraction  = now_from_start(start_time)/self._timeout
           print (self._plan_fraction)
           #print (self._timeout)
           #print (self.ground_detector.ground_position.x)
           #distance =  euclidean_distance (self.ground_detector.ground_position, goal_pose)
           #print (distance)
           print ('_______________')
           
        success = self.trajectory_async_executer.success() and self.trajectory_async_executer.wait()
            
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


if __name__ == '__main__':
    rospy.init_node('GuardedMove')
    server = GuardedMoveActionServer(rospy.get_name())
    rospy.spin()
