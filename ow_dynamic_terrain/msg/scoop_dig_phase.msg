std_msgs/Header header
bool digging        # returns true when scoop appears to be performing a
                    # digging motion
uint8 phase         # returns an enumerate that represents what phase of a dig
                    # the scoop is estimated to be in

# phase enumeration constants
uint8 NOT_DIGGING = 0
uint8 SINKING     = 1
uint8 PLOWING     = 2
uint8 RETRACTING  = 3

