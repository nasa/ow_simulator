# System Fault Status Mask

# The 'value' field is the fault classification, which is the
# following enumeration, implemented in FaultDetector.h.  The
# associated message is a bitmask that can represent any subset of
# these faults occuring at the same time.

# NO_FAULT = 0,
# SYSTEM = 1,
# ARM_GOAL_ERROR = 2,
# ARM_EXECUTION_ERROR = 4,
# TASK_GOAL_ERROR = 8,
# CAMERA_GOAL_ERROR = 16,
# CAMERA_EXECUTION_ERROR = 32,
# PAN_TILT_GOAL_ERROR = 64,
# PAN_TILT_EXECUTION_ERROR = 128,
# LANDER_EXECUTION_ERROR = 256,
# POWER_EXECUTION_ERROR = 512

Header header
uint64 value
