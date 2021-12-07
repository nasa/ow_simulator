// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <ignition/math.hh>
#include "TerrainBrush.h"

using namespace cv;
using ignition::math::clamp;
using namespace ow_dynamic_terrain;

Mat TerrainBrush::circle(float outer_radius, float inner_radius, float weight)
{
  auto i_outer_radius = static_cast<int>(ceil(outer_radius));
  auto center = Point2i(i_outer_radius, i_outer_radius);  // center with respect to the image.
  auto result = Mat(2 * i_outer_radius, 2 * i_outer_radius, CV_32FC1);

  result.forEach<float>([=, &center](float &pixel_value, const int pixel_index[]) {
    auto dx = pixel_index[1] - center.x;
    auto dy = pixel_index[0] - center.y;
    auto dist = sqrtf(dx * dx + dy * dy);

    auto falloff_weight = 1.0f;  // weight used to interpolate between inner and outer rings of the circle.
    if (dist > inner_radius)
    {
      falloff_weight = clamp((dist - inner_radius) / (outer_radius - inner_radius), 0.0f, 1.0f);
      falloff_weight = 1.0f - (falloff_weight * falloff_weight);
    }

    pixel_value = falloff_weight * weight;
  });

  return result;
}

float TerrainBrush::intersectEllipseLineX(float a, float b, float m)
{
  return a * b / sqrtf(b * b + m * m * a * a);
}

Mat TerrainBrush::ellipse(float outer_radius_a, float inner_radius_a, float outer_radius_b, float inner_radius_b,
                          float weight)
{
  auto i_outer_radius_a = static_cast<int>(ceil(outer_radius_a));
  auto i_outer_radius_b = static_cast<int>(ceil(outer_radius_b));
  auto center = Point2i(i_outer_radius_a, i_outer_radius_b);  // center with respect to the image.
  auto result = Mat(2 * i_outer_radius_b, 2 * i_outer_radius_a, CV_32FC1);

  auto inner_radius_a_sq = inner_radius_a * inner_radius_a;
  auto inner_radius_b_sq = inner_radius_b * inner_radius_b;

  result.forEach<float>([=, &center](float &pixel_value, const int pixel_index[]) {
    auto dx = pixel_index[1] - center.x;
    auto dy = pixel_index[0] - center.y;
    auto inner_dist_sq = inner_radius_b_sq * dx * dx + inner_radius_a_sq * dy * dy;

    auto falloff_weight = 1.0f;  // weight used to interpolate between inner and outer rings
    if (inner_dist_sq > inner_radius_a_sq * inner_radius_b_sq)
    {
      if (dx != 0)
      {
        auto m = static_cast<float>(dy) / static_cast<float>(dx);
        auto x1 = TerrainBrush::intersectEllipseLineX(inner_radius_a, inner_radius_b, m);
        auto x2 = TerrainBrush::intersectEllipseLineX(outer_radius_a, outer_radius_b, m);
        falloff_weight = (abs(dx) - x1) / (x2 - x1);
      }
      else
      {
        falloff_weight = (abs(dy) - inner_radius_b) / (outer_radius_b - inner_radius_b);
      }

      falloff_weight = clamp(falloff_weight, 0.0f, 1.0f);
      falloff_weight = 1.0f - (falloff_weight * falloff_weight);
    }

    pixel_value = falloff_weight * weight;
  });

  return result;
}