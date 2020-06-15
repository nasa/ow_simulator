geometry_msgs/Point32 position  # position in the real world, position is the center of the of the patch
                                # z value if set introduces a z bias to genetated height values
float32 orientation             # rotation angle of ellipse around z axis (measured in degrees)
sensor_msgs/Image patch         # an image representing the patch of terrain to update
string merge_method             # decides how to merge supplied values with height values of the terrain
                                # current choices: keep, replace, add, sub, min, max. If not specified default is: add