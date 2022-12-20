#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
from sensor_msgs.msg import JointState
from owl_msgs.msg import ArmJointAccelerations, ArmJointPositions, ArmJointTorques, ArmJointVelocities, PanTiltPosition

class StatePublisher(object):

  def __init__(self, name):
    self._joint_acc = ArmJointAccelerations()
    self._joint_pos = ArmJointPositions()
    self._joint_tor = ArmJointTorques()
    self._joint_vel = ArmJointVelocities()
    self._pan_tilt = PanTiltPosition()

    self._subscriber = rospy.Subscriber(
            '/joint_states', JointState, self._handle_joint_states)
    self._arm_joint_accelerations_pub = rospy.Publisher(
            '/arm_joint_accelerations', ArmJointAccelerations, queue_size=10)
    self._arm_joint_positions_pub = rospy.Publisher(
            '/arm_joint_positions', ArmJointPositions, queue_size=10)
    self._arm_joint_torques_pub = rospy.Publisher(
            '/arm_joint_torques', ArmJointTorques, queue_size=10)
    self._arm_joint_velocities_pub = rospy.Publisher(
            '/arm_joint_velocities', ArmJointVelocities, queue_size=10)
    self._pan_tilt_pub = rospy.Publisher(
            '/pan_tilt_position', PanTiltPosition, queue_size=10)

  def _handle_joint_states(self, data):
    """
    :type data: sensor_msgs.msg.JointState
    """
    # Check for compatible input and output array sizes
    if len(data.position) != len(self._joint_pos.value) + len(self._pan_tilt.value) :
      rospy.logerr_throttle(1, 'StatePublisher: Array size mismatch')
      return

    # Offset to end of pan/tilt data and start of arm data
    OFFSET = 2

    dur = data.header.stamp - self._joint_acc.header.stamp
    dur_float = dur.to_sec()
    # Only publish accelerations if time is moving forward (this should always happen)
    if dur_float > 0 :
      self._joint_acc.header = data.header
      for i in range(0, len(self._joint_acc.value)):
        self._joint_acc.value[i] = (data.velocity[i + OFFSET] - self._joint_vel.value[i]) / dur_float
      self._arm_joint_accelerations_pub.publish(self._joint_acc)

    self._joint_pos.header = data.header
    self._joint_pos.value = data.position[OFFSET : len(data.position)]
    self._arm_joint_positions_pub.publish(self._joint_pos)

    self._joint_tor.header = data.header
    self._joint_tor.value = data.effort[OFFSET : len(data.effort)]
    self._arm_joint_torques_pub.publish(self._joint_tor)

    self._joint_vel.header = data.header
    self._joint_vel.value = data.velocity[OFFSET : len(data.velocity)]
    self._arm_joint_velocities_pub.publish(self._joint_vel)

    self._pan_tilt.header = data.header
    self._pan_tilt.value = data.position[0 : OFFSET]
    self._pan_tilt_pub.publish(self._pan_tilt)

if __name__ == '__main__':
    node_name = 'state_publisher'
    rospy.init_node(node_name)
    state_publisher = StatePublisher(node_name)
    rospy.spin()
