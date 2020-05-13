#include <OgreVector3.h>
#include <gazebo/rendering/Conversions.hh>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "TerrainModifier.h"

using namespace std;
using namespace Ogre;
using namespace gazebo;
using namespace rendering;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv_bridge;


void TerrainModifier::modify(Heightmap* heightmap,
                             const ow_dynamic_terrain::modify_terrain_circle::ConstPtr& msg,
                             function<float(long, long)> get_height_value,
                             function<void(long, long, float)> set_height_value)
{
  if (msg->operation != "lower" && msg->operation != "raise")
  {
    gzerr << "DynamicTerrain: Unknown terrain operation [" << msg->operation << "]" << endl;
    return;
  }

  auto raise_operation = msg->operation == "raise";  // otherwise it is lower

  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return;
  }

  if (msg->inner_radius > msg->outer_radius)
  {
    gzerr << "DynamicTerrain: inner_radius can't exceed outer_radius value!" << endl;
    return;
  }

  auto _terrain_position = Vector3(msg->position.x, msg->position.y, 0);
  Vector3 heightmap_position;
  terrain->getTerrainPosition(_terrain_position, &heightmap_position);

  auto size = static_cast<int>(terrain->getSize());
  auto left = max(int((heightmap_position.x - msg->outer_radius) * size), 0);
  auto top = max(int((heightmap_position.y - msg->outer_radius) * size), 0);
  auto right = min(int((heightmap_position.x + msg->outer_radius) * size), size);
  auto bottom = min(int((heightmap_position.y + msg->outer_radius) * size), size);

  for (auto y = top; y <= bottom; ++y)
    for (auto x = left; x <= right; ++x)
    {
      auto ts_x_dist = x / static_cast<double>(size) - heightmap_position.x;
      auto ts_y_dist = y / static_cast<double>(size) - heightmap_position.y;
      auto dist = sqrt(ts_y_dist * ts_y_dist + ts_x_dist * ts_x_dist);

      auto inner_weight = 1.0;
      if (dist > msg->inner_radius)
      {
        inner_weight = ignition::math::clamp(
          (dist - msg->inner_radius) / (msg->outer_radius - msg->inner_radius), 0.0, 1.0);
        inner_weight = 1.0 - (inner_weight * inner_weight);
      }

      auto added_height = inner_weight * msg->weight;
      auto new_height = get_height_value(x, y) +
       (raise_operation ? +added_height : -added_height);

      set_height_value(x, y, new_height);
    }

  gzlog << "DynamicTerrain: circle " << msg->operation << " operation at ("
    << msg->position.x << ", " << msg->position.y << ")" << endl;
}

void TerrainModifier::modify(Heightmap* heightmap,
                             const ow_dynamic_terrain::modify_terrain_patch::ConstPtr& msg,
                             function<float(long, long)> get_height_value,
                             function<void(long, long, float)> set_height_value)
{
  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return;
  }

  if (msg->patch.encoding != "32FC1" && msg->patch.encoding != "64FC1")
  {
    gzerr << "DynamicTerrain: Only {32FC1, 64FC1} formats are supported" << endl;
    return;
  }

  CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = toCvShare(msg->patch, msg, image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    gzerr << "DynamicTerrain: cv_bridge exception: %s" << e.what() << endl;
    return;
  }

  auto _terrain_position = Vector3(msg->position.x, msg->position.y, 0);
  Vector3 heightmap_position;
  terrain->getTerrainPosition(_terrain_position, &heightmap_position);

  auto size = static_cast<int>(terrain->getSize());
  auto left = max(int(heightmap_position.x * size), 0);
  auto top = max(int(heightmap_position.y * size), 0);
  auto right = min(left + static_cast<int>(msg->patch.width - 1), size);
  auto bottom = min(top + static_cast<int>(msg->patch.height - 1), size);

  for (auto y = top; y <= bottom; ++y)
    for (auto x = left; x <= right; ++x)
    {
      auto row = y - top;
      auto col = x - left;
      auto diff_height = cv_ptr->image.at<float>(row, col);
      auto new_height = get_height_value(x, y) + diff_height * msg->z_scale;
      set_height_value(x, y, new_height);
    }

  gzlog << "DynamicTerrain: patch applied at ("
    << msg->position.x << ", " << msg->position.y << ")" << endl;
}