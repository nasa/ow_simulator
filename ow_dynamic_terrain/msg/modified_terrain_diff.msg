std_msgs/Header header
sensor_msgs/Image result        # an image containing resulting heights within
                                # the modification area
sensor_msgs/Image diff          # an image containing changes in height
geometry_msgs/Point32 position  # position in the world frame where the terrain
                                # modification took place 
float32 height                  # height of image in world units
float32 width                   # width of image in world units
