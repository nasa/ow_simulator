# Camera Fault Status Mask

# The 'value' field is the fault classification as the following enumeration.
# The associated message is a bitmask that can represent any subset of
# these faults occuring at the same time.
# Bitwise operations on uints should be safe as ROS messages are serialized in
# little endian format.

Header header
int32 value

# Constant bitset of error types/flags to set as value
uint32 NONE     = 0
uint32 NO_IMAGE = 0
