geometry_msgs/Point32 position  # position in the real world, where x, y are at
                                # the center of circle
float32 outer_radius            # beyond outer_radius operation has no effect
float32 inner_radius            # within inner radius the provided weight is
                                # applied at 100%, while values are interpolated
                                # between inner and outer radii
float32 weight                  # weight of the operation on height values