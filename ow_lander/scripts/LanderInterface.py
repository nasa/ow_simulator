#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import ow_lander.msg
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gazebo_msgs.msg import LinkStates
import math
import constants
from sensor_msgs.msg import JointState
from threading import Lock
import numpy as np
from std_msgs.msg import Float64
import tf2_ros
from moveit_msgs.srv import GetPositionFK
#from geometry_msgs.msg import Point, WrenchStamped


class MoveItInterface(object):

    def __init__(self):
        """ Initialize robot commander"""
        super(MoveItInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        move_arm = moveit_commander.MoveGroupCommander("arm")
        move_limbs = moveit_commander.MoveGroupCommander("limbs")
        move_grinder = moveit_commander.MoveGroupCommander("grinder")
        self.move_arm = move_arm
        self.move_limbs = move_limbs
        self.move_grinder = move_grinder
        self.robot = robot
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
        # forward kinematics: enable forward kinematics service from moveit
        rospy.wait_for_service('compute_fk')
        try:
            self.moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        except rospy.ServiceException as e:
            rospy.logerror("Service call failed: %s" % e)

class LinkStateSubscriber:

    def __init__(self):
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._link_value = None
        self.subscriber = rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self._handle_link_states)

    def _handle_link_states(self, data):
        # position of the end of scoop can be obtained either from raw Gazebo readings
        # of Link states or from tf2. TF2 method is commented out . Link State method is
        # being used in the current implementation
        """
        :type data: gazebo_msgs.msg._LinkStates.LinkStates
        """
        try:
            idx = data.name.index("lander::l_scoop_tip")
        except ValueError:
            rospy.logerr_throttle(
                1, "LanderInterface: lander::l_scoop_tip not found in link_states")
            return
        self._link_value = (data.pose[idx].position)

        # try:
        # t = self._buffer.lookup_transform(
        # "base_link", "l_scoop_tip", rospy.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #rospy.logwarn("LanderInterface: tf2 raised an exception")
        # return None
        # return t.transform.translation
        #self._link_value = t.transform.translation


class ArmJointStateSubscriber:

    def __init__(self):
        self._joint_value = None
        self._joint_lock = Lock()
        self._angles, self._velocities = {}, {}
        self.subscriber = rospy.Subscriber(
            "/joint_states", JointState, self._handle_joint_states)

    def _handle_joint_states(self, data):
        # position of joints are computed using Joint State method
        """
        :type data: sensor_msgs.msg.JointState
        """

        with self._joint_lock:
            for name, position, velocity in zip(data.name, data.position, data.velocity):
                self._angles[name] = position
                self._velocities[name] = velocity

    def get_joint_angles(self):
        # returns current joint angles
        with self._joint_lock:
            # all joints = ['j_ant_pan', 'j_ant_tilt', 'j_dist_pitch', 'j_grinder',
            #               'j_hand_yaw', 'j_prox_pitch', 'j_scoop_yaw', 'j_shou_pitch', 'j_shou_yaw']
            # subset used in arm controller in order
            joints_ret = ['j_shou_yaw','j_shou_pitch','j_prox_pitch','j_dist_pitch','j_hand_yaw', 'j_scoop_yaw']              
            try:
                return np.array([self._angles[k] for k in joints_ret])
            except KeyError:
                return None
