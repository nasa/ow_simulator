# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from math import sqrt, isclose, acos, sin
from geometry_msgs.msg import Quaternion, Vector3, Point
import tf.transformations

def _types_compatible(a, b):
  return (type(a) == type(b)) \
      or (type(a) == Vector3 and type(b) == Point) \
      or (type(a) == Point and type(b) == Vector3)

def add(a, b):
  """Adds two vectors
  a -- geometry_msgs Vector3 or Point
  b -- geometry_msgs Vector3 or Point
  returns a Vector3 that represents a + b
  """
  return Vector3(a.x + b.x, a.y + b.y, a.z + b.z)

def subtract(a, b):
  """Subtracts two vectors
  a -- geometry_msgs Vector3 or Point
  b -- geometry_msgs Vector3 or Point
  returns a Vector3 that represents a - b
  """
  return Vector3(a.x - b.x, a.y - b.y, a.z - b.z)

def scalar_multiply(a, b):
  """Computes the multiplication of the scalar a to the vector b
  a -- float
  b -- geometry_msgs Vector3 or Point
  returns a vector that is the result of ab
  """
  return type(b)(a * b.x, a * b.y, a * b.z)

def quaternion_inverse(a):
  """Compute the inverse of a quaternion
  a -- geometry_msgs Quaternion
  returns the quaternion inverse of a
  """
  return Quaternion(-a.x, -a.y, -a.z, a.w)

def quaternion_multiply(a, b):
  """Compute the product of two quaternions multiplied together (a * b)
  a -- geometry_msgs Quaternion
  b -- geometry_msgs Quaternion
  returns quaternion product of a and b
  """
  return Quaternion(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z)

def quaternion_rotate(q, v):
  """Perform a passive rotation of a vector using a quaternion
  q -- geometry_msgs Quaternion -- Should be a unit quaternion
  v -- geometry_msgs Pointer/Vector3
  returns v rotated by q as the same type as v
  """
  v_quat = Quaternion(v.x, v.y, v.z, 0)
  q_inv = quaternion_inverse(q)
  product = quaternion_multiply(quaternion_multiply(q, v_quat), q_inv)
  return type(v)(product.x, product.y, product.z)

def dot(a, b):
  """Computes dot product between vectors/Quaternions a and b (a * b)
  a -- geometry_msgs Vector3, Point, or Quaternions
  b -- geometry_msgs Vector3, Point, or Quaternions
  returns a float that is the result of a * b
  """
  assert(_types_compatible(a, b))
  dot = a.x * b.x + a.y * b.y + a.z * b.z
  if hasattr(a, 'w'):
    dot += a.w * b.w
  return dot

def cross(a, b):
  """Computes the cross product between vectors a and b (a x b)
  a -- geometry_msgs Vector3 or Point
  b -- geometry_msgs Vector3 or Point
  returns a vector that is the result of a x b
  """
  assert(_types_compatible(a, b))
  return type(a)(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)

def norm_squared(v):
  """Computes the squared norm (or length) of a vector or quaternion
  v -- geometry_msgs Vector3, Point, or Quaternion
  returns the squared norm of v
  """
  return dot(v, v)

def norm(v):
  """Computes the norm (or length) of a vector or quaternion
  v -- geometry_msgs Vector3, Point, or Quaternion
  returns the norm of v
  """
  return sqrt(norm_squared(v))

def normalize(v):
  """Normalizes a vector or quaternion
  v -- geometry_msgs Vector3, Point, or Quaternion
  returns the normalized version of v
  """
  n = norm(v)
  assert(n != 0)
  if hasattr(v, 'w'):
    return type(v)(v.x / n, v.y / n, v.z / n, v.w / n)
  else:
    return type(v)(v.x / n, v.y / n, v.z / n)

def is_normalized(v):
  """Normalization check
  v -- geometry_msgs Vector3 or Point
  returns true if v is normalized
  """
  return norm_squared(v) == 1.0

def distance(a, b):
  """Compute the distance between vectors a and b, same as norm(b - a)
  a -- geometry_msgs Vector3 or Point
  b -- geometry_msgs Vector3 or Point
  returns a float that represents the distance between two vectors
  """
  return norm(subtract(b, a))

def orthogonal(v):
  """Returns an orthogonal vector to v
  v -- geometry_msgs Vector3 or Point
  returns the orthogonal vector of v
  """
  normalized = normalize(v)
  t = type(v)
  x = abs(normalized.x)
  y = abs(normalized.y)
  z = abs(normalized.z)
  basis = None
  if x < y:
    basis = t(1, 0, 0) if x < z else t(0, 0, 1)
  else:
    basis = t(0, 1, 0) if y < z else t(0, 0, 1)
  return cross(normalized, basis)

def quaternion_from_euler(x, y, z):
  """A version of tf.transformation's quaternion_from_euler that returns a
  geometry_msgs Quaternion
  x -- Radian rotation around x (roll)
  y -- Radian rotation around y (pitch)
  z -- Radian rotation around z (yaw)
  returns a geometry_msgs Quaternion
  """
  return Quaternion(*list(tf.transformations.quaternion_from_euler(x, y, z)))

def euler_from_quaternion(q):
  """A version of tf.transformation's euler_from_quaternion that accepts a
  geometry_msgs Quaternion
  q -- geometry_msgs Quaternion
  returns a 3-tuple of Euler angles (x, y, z)
  """
  return tuple(tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]))

def slerp_quaternion(q1, q2, t):
  """Computes a spherical linear interpolation (slerp) between two quaternions.
  This wraps the method quaternion_slerp from tf.transformation so that it
  accepts and returns geometry_msgs types.
  q1 -- geometry_msgs Quaternion -- The value returned when t=0
  q2 -- geometry_msgs Quaternion -- The value returned when t=1
  t  -- Fraction between q1 and q1. 0 <= t <= 1
  returns a geometry_msgs quaternion between q1 and q1 by a fractional amount t
  """
  return Quaternion(
    *list(
      tf.transformations.quaternion_slerp(
        [q1.x, q1.y, q1.z, q1.w], [q2.x, q2.y, q2.z, q2.w], t
      )
    )
  )

def slerp(a, b, t):
  """Compute a spherical linear interpolation (slerp) between two vectors.
  a -- geometry_msgs Point/Vector3 -- The value returned when t=0
  b -- geometry_msgs Point/Vector3 -- The value returned when t=1
  t -- Fraction between a and b. 0 <= t <= 1
  returns a geometry_msgs Point/Vector3 between a and b by a fractional amount t
  """
  # handle case where points of contact are within floating-precision error of
  # each other
  if isclose(distance(a, b), 0):
    return a
  # compute arc length between points of contact
  al = acos(dot(normalize(a), normalize(b)))
  # compute scalar weight for a and b respectively
  weight_a = sin((1 - t) * al) / sin(al)
  weight_b = sin(t * al) / sin(al)
  return add(scalar_multiply(weight_a, a), scalar_multiply(weight_b, b))

def quaternion_rotation_between(a, b):
  """Computes the quaternion rotation between the vectors a and b.
  a -- geometry_msgs Vector3 or Point
  b -- geometry_msgs Vector3 or Point
  returns a quaternion that represents a rotation from a to b
  """
  a = normalize(a)
  b = normalize(b)
  k = dot(a, b)
  ab_norm = sqrt(norm_squared(a) * norm_squared(b))
  if isclose(k / ab_norm, -1): # special case of a = -b
    o = normalize(orthogonal(a))
    return Quaternion(o.x, o.y, o.z, 0)
  w = k + ab_norm
  v = cross(a, b)
  return normalize(Quaternion(v.x, v.y, v.z, w))

def vectors_approx_equivalent(a, b, tolerance):
  """Check if two vectors are nearly the same.
  a -- geometry_msgs Point/Vector3
  b -- geometry_msgs Point/Vector3
  tolerance -- The maximal distance positions can differ by
  returns True if the difference between a and b are below the tolerance
  """
  return distance(a, b) <= tolerance

def poses_approx_equivalent(pose1, pose2, linear_tolerance, angular_tolerance):
  """Check if two poses are nearly the same position and orientation.
  pose1 -- geometry_msgs Pose
  pose2 -- geometry_msgs Pose to be checked against pose1
  linear_tolerance  -- The maximal distance positions can differ by
  angular_tolerance -- The maximal spherical distance orientations can differ by
                       (radians)
  returns True if the difference between pose1 and pose2's position and
  orientations are below their respective tolerances. False otherwise.
  """
  p1 = pose1.position
  p2 = pose2.position
  if vectors_approx_equivalent(p1, p2, linear_tolerance):
    o1 = pose1.orientation
    o2 = pose2.orientation
    # check that the geodesic norm between the 2 quaternions is below tolerance
    dp = dot(o1, o2)
    if acos(2*dp*dp-1) <= angular_tolerance:
      return True
  return False
