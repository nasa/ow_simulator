string operation                # valid operations: lower, raise
geometry_msgs/Point position    # position in the real world, where x, y are at
                                # the center of circle
float64 outer_radius            # beyond outer_radius operation has no effect
float64 inner_radius            # within inner radius the provided weight is
                                # applied at 100%, while values are interpolated
                                # between inner and outer radii
float64 weight                  # weight of the operation on height values