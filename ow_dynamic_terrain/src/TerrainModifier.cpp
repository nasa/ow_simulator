#include <OgreVector3.h>
#include <gazebo/rendering/Conversions.hh>
#include "TerrainModifier.h"

using namespace std;
using namespace Ogre;
using namespace gazebo;
using namespace rendering;
using namespace geometry_msgs;

void TerrainModifier::modify(Heightmap* heightmap, const string& operation,
                             const Point& terrain_position,
                             double outer_radius, double inner_radius, double weight,
                             function<float(long, long)> get_height_value,
                             function<void(long, long, float)> set_height_value)
{
  if (operation != "lower" && operation != "raise")
  {
    gzerr << "DynamicTerrain: Unknown terrain operation [" << operation << "]" << endl;
    return;
  }

  auto raise_operation = operation == "raise";  // otherwise it is lower

  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return;
  }

  if (inner_radius > outer_radius)
  {
    gzerr << "DynamicTerrain: inner_radius can't exceed outer_radius value!" << endl;
    return;
  }

  auto _terrain_position = Vector3(terrain_position.x, terrain_position.y, 0);
  Vector3 heightmap_position;
  terrain->getTerrainPosition(_terrain_position, &heightmap_position);

  auto size = static_cast<int>(terrain->getSize());
  auto left = max(int((heightmap_position.x - outer_radius) * size), 0);
  auto top = max(int((heightmap_position.y - outer_radius) * size), 0);
  auto right = min(int((heightmap_position.x + outer_radius) * size), size);
  auto bottom = min(int((heightmap_position.y + outer_radius) * size), size);

  for (auto y = top; y <= bottom; ++y)
    for (auto x = left; x <= right; ++x)
    {
      auto ts_x_dist = x / static_cast<double>(size) - heightmap_position.x;
      auto ts_y_dist = y / static_cast<double>(size) - heightmap_position.y;
      auto dist = sqrt(ts_y_dist * ts_y_dist + ts_x_dist * ts_x_dist);

      auto inner_weight = 1.0;
      if (dist > inner_radius)
      {
        inner_weight = ignition::math::clamp(
          (dist-inner_radius) / (outer_radius-inner_radius), 0.0, 1.0);
        inner_weight = 1.0 - (inner_weight * inner_weight);
      }

      auto added_height = inner_weight * weight;
      auto new_height = get_height_value(x, y) +
       (raise_operation ? +added_height : -added_height);

      set_height_value(x, y, new_height);
    }

  gzlog << "DynamicTerrain: circle " << operation << " operation at ("
    << terrain_position.x << ", " << terrain_position.y << ")" << std::endl;
}

void TerrainModifier::modify(Heightmap* heightmap,
                             const Point& terrain_position,
                             const sensor_msgs::Image& patch,
                             double z_scale,
                             function<float(long, long)> get_height_value,
                             function<void(long, long, float)> set_height_value)
{
  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return;
  }

  if (patch.width != patch.step)
  {
    gzerr << "DynamicTerrain: patch step does't match width!"
     << " For now only mono8 format is supported." << endl;
    return;
  }

  auto _terrain_position = Vector3(terrain_position.x, terrain_position.y, 0);
  Vector3 heightmap_position;
  terrain->getTerrainPosition(_terrain_position, &heightmap_position);

  auto size = static_cast<int>(terrain->getSize());
  auto left = max(int(heightmap_position.x * size), 0);
  auto top = max(int(heightmap_position.y * size), 0);
  auto right = min(left + static_cast<int>(patch.width - 1), size);
  auto bottom = min(top + static_cast<int>(patch.height - 1), size);

  for (auto y = top; y <= bottom; ++y)
    for (auto x = left; x <= right; ++x)
    {
      auto row = y - top;
      auto col = x - left;
      auto diff_height = patch.data[patch.step * row + col] / 255.0;
      auto new_height = get_height_value(x, y) + diff_height * z_scale;
      set_height_value(x, y, new_height);
    }

  gzlog << "DynamicTerrain: patch applied at ("
    << terrain_position.x << ", " << terrain_position.y << ")" << std::endl;
}