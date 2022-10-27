#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import ow_lander.msg
from constants import PAN_MIN, PAN_MAX, PAN_TOLERANCE
from constants import TILT_MIN, TILT_MAX, TILT_TOLERANCE
from utils import normalize_radians, in_range
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from moveit_commander.conversions import pose_to_list

class AntennaPanTiltActionServer(object):

    def __init__(self, name):
        self._action_name = name
        # Action Feedback/Result
        self._fdbk = ow_lander.msg.AntennaPanTiltFeedback()
        self._result = ow_lander.msg.AntennaPanTiltResult()
        self._tilt_pub = rospy.Publisher(
            '/ant_tilt_position_controller/command', Float64, queue_size=10)
        self._pan_pub = rospy.Publisher(
            '/ant_pan_position_controller/command', Float64, queue_size=10)
        self._subscriber = rospy.Subscriber(
            "/joint_states", JointState, self._handle_joint_states)
        # KMD: the following two lines seem questionable since these
        # members are assigned by the previous subscriber.
        self._pan_value = None
        self._tilt_value = None
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    ow_lander.msg.AntennaPanTiltAction,
                                                    execute_cb=self.on_antenna_action,
                                                    auto_start=False)
        self._server.start()

    def _handle_joint_states(self, data):
        # position of pan and tlt of the lander is obtained from JointStates
        """
        :type data: sensor_msgs.msg.JointState
        """
        try:
            id_pan = data.name.index("j_ant_pan")
            id_tilt = data.name.index("j_ant_tilt")
        except ValueError:
            rospy.logerr_throttle(
                1, "LanderInterface: j_ant_pan or j_ant_tilt not found in joint_states")
            return
        self._tilt_value = normalize_radians(data.position[id_tilt])
        self._pan_value = normalize_radians(data.position[id_pan])
        
        # KMD: the rest of this function is for debugging only.
        pan_correction = abs(data.position[id_pan] - self._pan_value);
        tilt_correction = abs(data.position[id_tilt] - self._tilt_value);
        if  pan_correction > PAN_TOLERANCE:
            rospy.logwarn_throttle (60, "Normalizing altered pan by %s"
                                    % pan_correction)
        if  tilt_correction > TILT_TOLERANCE:
            rospy.logwarn_throttle (60, "Normalizing altered tilt by %s"
                                    % tilt_correction)

    def _update_feedback(self):
        #self._ls =  self._current_link_state._link_value
        self._fdbk.pan_position = self._pan_value
        self._fdbk.tilt_position = self._tilt_value
        self._server.publish_feedback(self._fdbk)

    def on_antenna_action(self, goal):
        # Check for valid range.  Note that action servers don't
        # support a "rejected" state, so using aborted if out of range.
        if not in_range(goal.pan, PAN_MIN, PAN_MAX):
            rospy.logwarn('Requested pan %s not within allowed limit, rejecting.'
                          % goal.pan)
            self._server.set_aborted(None, 'invalid pan value')
            return
        if not in_range(goal.tilt, TILT_MIN, TILT_MAX):
            rospy.logwarn('Requested tilt %s not within allowed limit, rejecting.'
                          % goal.tilt)
            self._server.set_aborted(None, 'invalid tilt value')
            return

        done = False
        while (done == False):
            # check that preempt has not been requested by the client or Faults
            if self._server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._server.set_preempted()
                done = False
                break

            self._pan_pub.publish(goal.pan)
            self._tilt_pub.publish(goal.tilt)
            self._update_feedback()

            # KMD (future work): add a timeout to this condition, and
            # decouple termination from success.
            if (abs(goal.pan - self._pan_value) < PAN_TOLERANCE and
                abs(goal.tilt - self._tilt_value) < TILT_TOLERANCE) :
                done = True
                # KMD: this check seems redundant, and the 'else' unreachable
                if done:
                    self._result.pan_position = self._fdbk.pan_position
                    self._result.tilt_position = self._fdbk.tilt_position
                    rospy.loginfo('%s: Succeeded' % self._action_name)
                    self._server.set_succeeded(self._result)
                else:
                    rospy.loginfo('%s: Failed' % self._action_name)
                    self._server.set_aborted(self._result)


if __name__ == '__main__':
    SERVER_NAME = 'AntennaPanTiltAction'
    rospy.init_node(SERVER_NAME)
    server = AntennaPanTiltActionServer(SERVER_NAME)
    rospy.spin()
