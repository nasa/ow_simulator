// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef TERRAIN_BRUSH_H
#define TERRAIN_BRUSH_H

#include <opencv2/opencv.hpp>

// TODO (optimization): provide an option to restrict computations to a rectangular window.

namespace ow_dynamic_terrain
{
class TerrainBrush
{
public:
  // returns an opencv matrix with CV_32FC1 type
  static cv::Mat circle(float outer_radius, float inner_radius, float weight);

  // returns an opencv matrix with CV_32FC1 type
  static cv::Mat ellipse(float outer_radius_a, float inner_radius_a, float outer_radius_b, float inner_radius_b,
                         float weight);
  
private:
  // solves for x the intersection of a ellipse and a line (where the line passes through the ellipse center)
  // for y value simply multiply by m
  // param a: radius a of the ellipse
  // param b: radius b of the ellipse
  // param m: line slope
  static float intersectEllipseLineX(float a, float b, float m);
};
}  // namespace ow_dynamic_terrain

#endif  // TERRAIN_BRUSH_H
