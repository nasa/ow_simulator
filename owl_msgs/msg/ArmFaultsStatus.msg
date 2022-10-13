# Arm Fault Status Mask

# The 'value' field is the fault classification, which is the
# following enumeration, implemented in FaultDetector.h.  The
# associated message is a bitmask that can represent any subset of
# these faults occuring at the same time.

# NONE = 0,
# HARDWARE = 1,
# TRAJECTORY_GENERATION = 2,
# COLLISION = 4,
# E_STOP = 8,
# POSITION_LIMIT = 16,
# JOINT_TORQUE_LIMIT = 32,
# VELOCITY_LIMIT = 64,
# NO_FORCE_DATA = 128,
# FORCE_TORQUE_LIMIT = 256

Header header
uint64 value
