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
      auto dist = sqrtf(ts_x_dist * ts_x_dist + ts_y_dist * ts_y_dist);

      auto intermediary_weight = 1.0f; // weight used between to interpolate between inner and outer rings.
      if (dist > inner_radius)
      {
        intermediary_weight = clamp((dist - inner_radius) / (outer_radius - inner_radius), 0.0f, 1.0f);
        intermediary_weight = 1.0f - (intermediary_weight * intermediary_weight);
      }

      result.at<float>(y, x) = intermediary_weight * weight;
    }

  return result;
}

float TerrainBrush::intersectEllipseLineX(float a, float b, float m)
{
  return a * b / sqrtf(b * b + m * m * a * a);
}

Mat TerrainBrush::ellipse(float outer_radius_a, float inner_radius_a, float outer_radius_b, float inner_radius_b, float weight)
{
  auto radius_a = static_cast<int>(ceil(outer_radius_a));
  auto radius_b = static_cast<int>(ceil(outer_radius_b));
  auto center = Point2i(radius_a, radius_b);  // center with respect to the image.
  auto result = Mat(2 * radius_b, 2 * radius_a, CV_32FC1);

  for (auto y = 0; y < result.rows; ++y)
    for (auto x = 0; x < result.cols; ++x)
    {
      auto ts_x_dist = x - center.x;
      auto ts_y_dist = y - center.y;
      auto inner_dist = sqrtf(inner_radius_b * inner_radius_b * ts_x_dist * ts_x_dist + inner_radius_a * inner_radius_a * ts_y_dist * ts_y_dist);

      auto intermediary_weight = 1.0f;  // weight used between to interpolate between inner and outer rings
      if (inner_dist > inner_radius_a * inner_radius_b)
      {
        if (ts_x_dist != 0)
        {
          auto m = float(ts_y_dist) / float(ts_x_dist);
          auto x1 = TerrainBrush::intersectEllipseLineX(inner_radius_a, inner_radius_b, m);
          auto x2 = TerrainBrush::intersectEllipseLineX(radius_a, radius_b, m);
          intermediary_weight =  clamp((abs(ts_x_dist) - x1) / (x2 - x1), 0.0f, 1.0f);
        }
        else
        {
          intermediary_weight = clamp((abs(ts_y_dist) - inner_radius_b) / (radius_b - inner_radius_b), 0.0f, 1.0f);
        }

        intermediary_weight = 1.0f - (intermediary_weight * intermediary_weight);
      }

      result.at<float>(y, x) = intermediary_weight * weight;
    }

  return result;
}