# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import tf2_ros

from ow_lander.common import Singleton, create_most_recent_header

# NOTE: These imports are not directly used, but importing them works around a
#       bug discussed here
# https://answers.ros.org/question/249433/tf2_ros-buffer-transform-pointstamped/
from tf2_geometry_msgs import *
# needed for Stamped types unsupported by tf2_geometry_msgs
from geometry_msgs.msg import Pose, Point, Vector3, Wrench

class FrameTransformer(metaclass = Singleton):
  """Wraps the tf2_ros interface for looking up transforms and performing
  transform operations on stamped geometry_msgs objects.
  """

  def __init__(self):
    self._buffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._buffer)

  def transform_present(self, geometry, source_frame, target_frame, \
      timeout=rospy.Duration(0.1)):
    """Performs a transform on a supported geometry_msgs object. Transform is
    performed in the present; use the transform for past transforms.
    geometry -- A geometry_msgs object. Supported: Point, Pose, Vector3, Wrench
    source_frame -- ROS identifier string from the frame being transformed from
    target_frame -- ROS identifier string for the frame to be transformed into
    returns a geometry_msgs object of the same type as provided, but transformed
    into the target_frame. None if transform call fails
    """
    if source_frame == target_frame:
      return geometry
    # convert geometry into a stamped type
    stamped = None
    if isinstance(geometry, Point):
      stamped = PointStamped(
        header=create_most_recent_header(source_frame), point=geometry)
    elif isinstance(geometry, Point):
      stamped = PoseStamped(
        header=create_most_recent_header(source_frame), pose=geometry)
    elif isinstance(geometry, Vector3):
      stamped = Vector3Stamped(
        header=create_most_recent_header(source_frame), vector=geometry)
    elif isinstance(geometry, Wrench):
      stamped = WrenchStamped(
        header=create_most_recent_header(source_frame), wrench=geometry)
    else:
      rospy.logerr(f"Unsupported geometry type {type(geometry)}")
      return None
    transformed = self.transform(stamped, target_frame, timeout=timeout)
    # return the same type as was provided
    if isinstance(transformed, PointStamped):
      return transformed.point
    elif isinstance(transformed, PoseStamped):
      return transformed.pose
    elif isinstance(transformed, Vector3Stamped):
      return transformed.vector
    elif isinstance(transformed, WrenchStamped):
      return transformed.wrench
    else:
      return None

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
