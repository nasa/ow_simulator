geometry_msgs/Point32 position  # position in the real world, where x, y are at the center of the ellipse
                                # z value if set introduces a z bias to genetated height values
float32 orientation             # rotation angle of ellipse around z axis measured in degrees
float32 outer_radius_a          # ellipse outer radius a
float32 outer_radius_b          # ellipse outer radius b
float32 inner_radius_a          # ellipse inner radius a
float32 inner_radius_b          # ellipse inner radius b
float32 weight                  # weight of the operation on height values
string merge_method             # decides how to merge generated values with height values of the terrain
                                # available choices: { keep, replace, add, sub, min, max, avg }
                                # If not specified default is: add