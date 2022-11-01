# System Fault Status Mask

# The 'value' field is the fault classification, which is the
# following enumeration, implemented as bitset constant expressions in
# ow_faults_detection/include/ow_faults_detection/FaultDetector.h
# The associated message is a bitmask that can represent any subset of
# these faults occuring at the same time.

Header header
uint64 value

# Constant bitset of error types/flags to set as value
uint64 NONE = 0
uint64 SYSTEM = 1
uint64 ARM_GOAL_ERROR = 2
uint64 ARM_EXECUTION_ERROR = 4
uint64 TASK_GOAL_ERROR = 8
uint64 CAMERA_GOAL_ERROR = 16
uint64 CAMERA_EXECUTION_ERROR = 32
uint64 PAN_TILT_GOAL_ERROR = 64
uint64 PAN_TILT_EXECUTION_ERROR = 128
uint64 LANDER_EXECUTION_ERROR = 256
uint64 POWER_EXECUTION_ERROR = 512
