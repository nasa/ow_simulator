#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class TrajectoryAsyncExecuter:
  def __init__(self):
    self._connected = False
    self._goal_time_tolerance = rospy.Time(0.1)

  def connect(self, controller):
    self._controller = controller
    self._client = actionlib.SimpleActionClient(
        "{}/follow_joint_trajectory".format(self._controller),
        FollowJointTrajectoryAction)
    self._connected = self._client.wait_for_server()
    if not self._connected:
      rospy.logerr(
          "Timed out waiting for {} joint trajectory action server".format(
              self._controller))
    else:
      rospy.loginfo(
          "Successfully connected to {} joint Trajectory action server".format(
              self._controller))
    return self._connected

  def execute(self, trajectory, done_cb=None, active_cb=None, feedback_cb=None):
    if not self._connected:
      rospy.logwarn_throttle(
          1, "TrajectoryAsyncExecuter: Client failed to connect to the action server .. nothing will be executed!")
      return False

    goal = FollowJointTrajectoryGoal()
    goal.goal_time_tolerance = self._goal_time_tolerance
    goal.trajectory = trajectory
    self._client.send_goal(goal, done_cb, active_cb, feedback_cb)

  def stop(self):
    self._client.cancel_goal()

  def wait(self, timeout=0):
    self._client.wait_for_result(timeout=rospy.Duration(timeout))

  def result(self):
    return self._client.get_result()
