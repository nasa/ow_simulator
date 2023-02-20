# Power Fault Status Mask

# The 'value' field is the fault classification as the following enumeration.
# The associated message is a bitmask that can represent any subset of
# these faults occuring at the same time.
# Bitwise operations on uints should be safe as ROS messages are serialized in
# little endian format.

Header header
uint64 value

# Constant bitset of error types/flags to set as value
uint64 NONE                        = 0
uint64 LOW_STATE_OF_CHARGE         = 1
uint64 INSTANTANEOUS_CAPACITY_LOSS = 2
uint64 THERMAL_FAULT               = 4
