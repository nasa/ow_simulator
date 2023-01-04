# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import tf2_ros

from ow_lander.common import Singleton

# NOTE: These imports are not directly used, but importing them works around a
#       bug discussed here
# https://answers.ros.org/question/249433/tf2_ros-buffer-transform-pointstamped/
from tf2_geometry_msgs import *

class FrameTransformer(metaclass = Singleton):
  """Wraps the tf2_ros interface for looking up transforms and performing
  transform operations on stamped geometry_msgs objects.
  """

  def __init__(self):
    self._buffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._buffer)

  def transform(self, stamped_type, target_frame, timeout=rospy.Duration(0.1)):
    """Performs a transform on a stamped geometry_msgs object
    stamped_type -- A stamped geometry_msgs object
    target_frame -- ROS identifier string for the frame to be transformed into
    returns a stamped object of the same type only transformed into target_frame
    or None if transform call fails
    """
    try:
      return self._buffer.transform(stamped_type, target_frame, timeout)
    except tf2_ros.TransformException as err:
      rospy.logerr(f"FrameTransfomer.transform failure: {str(err)}")
      return None

  def lookup_transform(self, target_frame, source_frame,
                       timestamp=rospy.Time(0), timeout=rospy.Duration(0.1)):
    """Computes a transform from the source frame to the target frame
    source_frame -- The frame from which the transform is computed
    target_frame -- The frame transformed into
    timestamp    -- Time on the ROS clock to look up frame transform. Too far in
                    the past will through an error. Defaults to most recent.
    returns a transform or None if lookup_transform call fails
    """
    try:
      return self._buffer.lookup_transform(target_frame, source_frame,
                                           timestamp, timeout)
    except tf2_ros.TransformException as err:
      rospy.logerr(f"FrameTransfomer.lookup_transform failure: {str(err)}")
      return None
