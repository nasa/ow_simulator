# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines classes for processing messages from certain ROS Topics"""

import rospy
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState

class LinkStateSubscriber:
  """Subscribes to /gazebo/link_states and returns the pose/position of a
  specified link
  """

  # The subscriber and message buffer will be shared by all instances
  # of this class within the same process
  _message_buffer = None
  _subscriber = None

  @classmethod
  def _on_link_states_msg(cls, message):
    # save the message
    cls._message_buffer = message

  def __init__(self, name):
    self._link_name = name
    if not LinkStateSubscriber._subscriber:
      # subscribe to link states only once
      LinkStateSubscriber._subscriber = rospy.Subscriber(
        "/gazebo/link_states", LinkStates,
        LinkStateSubscriber._on_link_states_msg
      )
    # TODO: block until first message is received

  def get_link_pose(self):
    if not LinkStateSubscriber._message_buffer:
      rospy.logerr(
        "Link position queried before /gazebo/link_states message received.")
      return
    try:
      idx = LinkStateSubscriber._message_buffer.name.index(self._link_name)
    except ValueError:
      rospy.logerr_once(f"{self._link_name} not found in link_states")
      return
    return LinkStateSubscriber._message_buffer.pose[idx]

  def get_link_position(self):
    return self.get_link_pose().position

class JointAnglesSubscriber:
  """Subscribes to /joint_states and returns the angle positions of a provided
  list of joints
  """

  _message_buffer = None
  _subscriber = None

  @classmethod
  def _on_joint_state_msg(self, message):
    # save the message
    self._message_buffer = message

  def __init__(self, names):
    self._joint_names = names
    if not JointAnglesSubscriber._subscriber:
      JointAnglesSubscriber._subscriber = rospy.Subscriber(
        "/joint_states", JointState, self._on_joint_state_msg
      )
    # TODO: block until first message is received

  def get_joint_positions(self):
    if not JointAnglesSubscriber._message_buffer:
      rospy.logerr(
        "Joint position queried before joint_states message received.")
      return
    angles = list()
    for name in self._joint_names:
      try:
        idx = JointAnglesSubscriber._message_buffer.name.index(name)
      except ValueError:
        rospy.logerr_once(f"{self._joint_names} not found in link_states")
        return
      angles.append(JointAnglesSubscriber._message_buffer.position[idx])
    return angles
