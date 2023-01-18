# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from math import sqrt, isclose
from geometry_msgs.msg import Quaternion

def _types_match(a, b):
  return type(a) == type(b)

def add(a, b):
  return type(a)(a.x + b.x, a.y + b.y, a.z + b.z)

def subtract(a, b):
  assert(_types_match(a, b))
  return type(a)(a.x - b.x, a.y - b.y, a.z - b.z)

def scalar_multiply(a, b):
  return type(b)(a * b.x, a * b.y, a * b.z)

def dot(a, b):
  assert(_types_match(a, b))
  dot = a.x * b.x + a.y * b.y + a.z * b.z
  if hasattr(a, 'w'):
    dot += a.w * b.w
  return dot

def cross(a, b):
  assert(_types_match(a, b))
  return type(a)(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)

def norm_squared(a):
  return dot(a,a)

def norm(a):
  return sqrt(norm_squared(a))

def normalize(v):
  n = norm(v)
  if hasattr(v, 'w'):
    return type(v)(v.x / n, v.y / n, v.z / n, v.w / n)
  else:
    return type(v)(v.x / n, v.y / n, v.z / n)

def is_normalized(v):
  return norm_squared(v) == 1.0

def orthogonal(v):
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

def quaternion_rotation_between(a, b):
  """Returns a quaternion that represents the rotation required to get from
  vector a to vector b.
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
