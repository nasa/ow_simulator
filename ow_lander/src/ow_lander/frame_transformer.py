# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import tf2_ros

from ow_lander.common import Singleton, create_header

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

  def transform_geometry(self, geometry, target_frame, source_frame,
                         timestamp=rospy.Time(0), timeout=rospy.Duration(0)):
    """Performs a transform on a supported geometry_msgs object. Transform is
    performed in the present; use the transform method for past transforms.
    geometry -- A geometry_msgs object. Supported: Point, Pose, Vector3, Wrench
    target_frame -- ROS identifier string for the frame to be transformed into
    source_frame -- ROS identifier string from the frame being transformed from
    timestamp -- The time of the desired transform. Assigning rospy.Time(0) will
                 request the most recent transform available in the cache. Even
                 older transforms may be requested, but be careful not to
                 request transforms further in the past than the cache saves.
                 Using ropsy.Time.now() will acquire the next (newest)
                 transform. If a rospy.Time.now() or a transform in the future
                 is used, a sufficiently long timeout must be set, or the frame
                 transform will not be found, resulting in an error.
                 default: rospy.Time(0)
    timeout -- The maximum time interval the method will block for while waiting
               for the requested transform to become available. Using
               rospy.Duration(0) will ensure the method returns immediately, and
               should only be used when requesting a timestamp in the past.
               default: rospy.Duration(0)
    returns a geometry_msgs object of the same type as provided, but transformed
    into the target_frame. None if transform call fails
    """
    if source_frame == target_frame:
      return geometry
    # convert geometry into a stamped type
    stamped = None
    if isinstance(geometry, Point):
      stamped = PointStamped(
        header=create_header(source_frame, timestamp), point=geometry)
    elif isinstance(geometry, Pose):
      stamped = PoseStamped(
        header=create_header(source_frame, timestamp), pose=geometry)
    elif isinstance(geometry, Vector3):
      stamped = Vector3Stamped(
        header=create_header(source_frame, timestamp), vector=geometry)
    elif isinstance(geometry, Wrench):
      stamped = WrenchStamped(
        header=create_header(source_frame, timestamp), wrench=geometry)
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

  def transform(self, stamped_type, target_frame, timeout=rospy.Duration(0)):
    """Performs a transform on a stamped geometry_msgs object
    stamped_type -- A stamped geometry_msgs object
    target_frame -- ROS identifier string for the frame to be transformed into
    timestamp -- The time of the desired transform. Assigning rospy.Time(0) will
                 request the most recent transform available in the cache. Even
                 older transforms may be requested, but be careful not to
                 request transforms further in the past than the cache saves.
                 Using ropsy.Time.now() will acquire the next (newest)
                 transform. If a rospy.Time.now() or a transform in the future
                 is used, a sufficiently long timeout must be set, or the frame
                 transform will not be found, resulting in an error.
                 default: rospy.Time(0)
    timeout -- The maximum time interval the method will block for while waiting
               for the requested transform to become available. Using
               rospy.Duration(0) will ensure the method returns immediately, and
               should only be used when requesting a timestamp in the past.
               default: rospy.Duration(0)
    returns a stamped object of the same type only transformed into target_frame
    or None if transform call fails
    """
    try:
      return self._buffer.transform(stamped_type, target_frame, timeout)
    except tf2_ros.TransformException as err:
      rospy.logerr(f"FrameTransfomer.transform failure: {str(err)}")
      return None

  def lookup_transform(self, target_frame, source_frame,
                       timestamp=rospy.Time(0), timeout=rospy.Duration(0)):
    """Computes a transform from the source frame to the target frame
    source_frame -- The frame from which the transform is computed
    target_frame -- The frame transformed into
    timestamp -- The time of the desired transform. Assigning rospy.Time(0) will
                 request the most recent transform available in the cache. Even
                 older transforms may be requested, but be careful not to
                 request transforms further in the past than the cache saves.
                 Using ropsy.Time.now() will acquire the next (newest)
                 transform. If a rospy.Time.now() or a transform in the future
                 is used, a sufficiently long timeout must be set, or the frame
                 transform will not be found, resulting in an error.
                 default: rospy.Time(0)
    timeout -- The maximum time interval the method will block for while waiting
               for the requested transform to become available. Using
               rospy.Duration(0) will ensure the method returns immediately, and
               should only be used when requesting a timestamp in the past.
               default: rospy.Duration(0)
    returns a transform or None if lookup_transform call fails
    """
    try:
      return self._buffer.lookup_transform(target_frame, source_frame,
                                           timestamp, timeout)
    except tf2_ros.TransformException as err:
      rospy.logerr(f"FrameTransfomer.lookup_transform failure: {str(err)}")
      return None

def initialize():
  """Initialize tf2 Buffer. Call this following rospy.init_node"""
  FrameTransformer()
