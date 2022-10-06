# System Fault Status Mask

# The 'value' field is the fault classification, which is the
# following enumeration, implemented in FaultDetector.h.  The
# associated message is a bitmask that can represent any subset of
# these faults occuring at the same time.

# NoFault = 0,
# System = 1,
# ArmGoalError = 2,
# ArmExecutionError = 4,
# TaskGoalError = 8,
# CameraGoalError = 16,
# CameraExecutionError = 32,
# PanTiltGoalError = 64,
# PanTiltExecutionError = 128,
# LanderExecutionError = 256,
# PowerExecutionError = 512

Header header
uint64 value
