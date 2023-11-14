# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines classes for processing messages from certain ROS Topics"""

import rospy
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState

def wait_for_message(message_buffer, timeout, frequency=10):
  r = rospy.Rate(frequency)
  for _i in range(int(timeout * frequency)):
    if message_buffer is not None:
      return True
    r.sleep()
  return False

MESSAGE_TIMEOUT = 30

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

  def get_link_pose(self):
    # block until first message is received
    if LinkStateSubscriber._message_buffer is None and \
        not wait_for_message(LinkStateSubscriber._message_buffer,
                             MESSAGE_TIMEOUT):
      rospy.logwarn("LinkStatesSubscriber did not receive a message for "\
                    f"{MESSAGE_TIMEOUT} seconds at startup. This may cause " \
                    "issues for some lander actions.")
      return None
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

  def _get_joint_index(self, name):
   return JointAnglesSubscriber._message_buffer.name.index(name)

  def _assert_first_joint_message_received(self):
    # block until first message is received
    if not JointAnglesSubscriber._message_buffer and \
        not wait_for_message(JointAnglesSubscriber._message_buffer,
                             MESSAGE_TIMEOUT):
      raise ValueError("JointAnglesSubscriber did not receive a message for "\
                    f"{MESSAGE_TIMEOUT} seconds at startup. This may cause " \
                    "issues for some lander actions.")

  def get_joint_positions(self):
    self._assert_first_joint_message_received()
    angles = list()
    for name in self._joint_names:
      idx = self._get_joint_index(name)
      angles.append(JointAnglesSubscriber._message_buffer.position[idx])
    return angles

  def get_joint_velocities(self):
    self._assert_first_joint_message_received()
    angles = list()
    for name in self._joint_names:
      idx = self._get_joint_index(name)
      angles.append(JointAnglesSubscriber._message_buffer.velocity[idx])
    return angles

