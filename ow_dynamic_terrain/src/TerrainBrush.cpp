#include <ignition/math.hh>
#include "TerrainBrush.h"

using namespace cv;
using ignition::math::clamp;
using namespace ow_dynamic_terrain;

Mat TerrainBrush::circle(float outer_radius, float inner_radius, float weight)
{
  auto radius = static_cast<int>(ceil(outer_radius));
  auto center = Point2i(radius, radius);  // center with respect to the image.
  auto result = Mat(2 * radius, 2 * radius, CV_32FC1);

  
  for (auto y = 0; y < result.rows; ++y)
    for (auto x = 0; x < result.cols; ++x)
    {
      auto ts_x_dist = x - center.x;
      auto ts_y_dist = y - center.y;
      auto dist = sqrtf(ts_y_dist * ts_y_dist + ts_x_dist * ts_x_dist);

      auto inner_weight = 1.0;
      if (dist > inner_radius)
      {
        inner_weight = clamp((dist - inner_radius) / (outer_radius - inner_radius), 0.0f, 1.0f);
        inner_weight = 1.0 - (inner_weight * inner_weight);
      }

      result.at<float>(y, x) = inner_weight * weight;
    }

  return result;
}
