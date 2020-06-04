#ifndef TERRAIN_BRUSH_H
#define TERRAIN_BRUSH_H


#include <opencv2/opencv.hpp>

// TODO: provide an option to clamp computation to a rectangular window

namespace ow_dynamic_terrain
{
class TerrainBrush
{
public:
  // returns an opencv matrix with CV_32FC1 type
  static cv::Mat circle(float outer_radius, float inner_radius, float weight);
};
}  // namespace ow_dynamic_terrain

#endif  // TERRAIN_BRUSH_H