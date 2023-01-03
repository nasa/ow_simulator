# Camera Fault Status Mask

# The 'value' field is the fault classification as the following enumeration.
# The associated message is a bitmask that can represent any subset of
# these faults occuring at the same time.
# Bitwise operations on uints should be safe as ROS messages are serialized in
# little endian format.

Header header
uint64 value

# Constant bitset of error types/flags to set as value
uint64 NONE     = 0
uint64 NO_IMAGE = 1
