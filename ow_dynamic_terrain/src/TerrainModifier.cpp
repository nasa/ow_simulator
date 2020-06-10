#include <OgreVector3.h>
#include <sensor_msgs/image_encodings.h>
#include <gazebo/common/Console.hh>
#include "TerrainModifier.h"
#include "OpenCV_Util.h"
#include "TerrainBrush.h"

using namespace std;
using namespace Ogre;
using namespace gazebo;
using namespace rendering;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;
using ignition::math::clamp;
using namespace ow_dynamic_terrain;

void TerrainModifier::modifyCircle(Heightmap* heightmap, const modify_terrain_circle::ConstPtr& msg,
                                   function<float(long, long)> get_height_value,
                                   function<void(long, long, float)> set_height_value)
{
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
  auto heightmap_position = Vector3();
  terrain->getTerrainPosition(_terrain_position, &heightmap_position);
  auto heightmap_size = static_cast<int>(terrain->getSize());
  auto image = TerrainBrush::circle(heightmap_size * msg->outer_radius, heightmap_size * msg->inner_radius, msg->weight);
  auto center = Point2i(heightmap_size * heightmap_position.x, heightmap_size * heightmap_position.y);
  applyImageToHeightmap(heightmap, center, image, get_height_value, set_height_value);

  gzlog << "DynamicTerrain: circle operation performed at (" << msg->position.x << ", "
        << msg->position.y << ")" << endl;
}

void TerrainModifier::modifyEllipse(Heightmap* heightmap, const modify_terrain_ellipse::ConstPtr& msg,
                                   function<float(long, long)> get_height_value,
                                   function<void(long, long, float)> set_height_value)
{
  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return;
  }

  if (msg->inner_radius_a > msg->outer_radius_a || msg->inner_radius_b > msg->outer_radius_b)
  {
    gzerr << "DynamicTerrain: inner_radius can't exceed outer_radius value!" << endl;
    return;
  }

  auto _terrain_position = Vector3(msg->position.x, msg->position.y, 0);
  auto heightmap_position = Vector3();
  terrain->getTerrainPosition(_terrain_position, &heightmap_position);
  auto heightmap_size = static_cast<int>(terrain->getSize());
  
  auto image = TerrainBrush::ellipse(
    heightmap_size * msg->outer_radius_a, heightmap_size * msg->inner_radius_a,
    heightmap_size * msg->outer_radius_b, heightmap_size * msg->inner_radius_b, msg->weight);
  auto center = Point2i(heightmap_size * heightmap_position.x, heightmap_size * heightmap_position.y);


  cv::imwrite("ellipse_image.png", OpenCV_Util::scaleImage_32FC1_To_8UC1(image));

  // TODO: apply msg->angle before applying it to the heightmap
  applyImageToHeightmap(heightmap, center, image, get_height_value, set_height_value);

  gzlog << "DynamicTerrain: ellipse operation performed at (" << msg->position.x << ", "
        << msg->position.y << ")" << endl;
}

  auto _terrain_position2 = Vector3(msg->position2.x, msg->position2.y, 0);
  auto heightmap_position2 = Vector3();
  terrain->getTerrainPosition(_terrain_position2, &heightmap_position2);

  // Check which should be top
  auto y_top = heightmap_position1.y < heightmap_position2.y ? heightmap_position1.y : heightmap_position2.y;
  auto y_bot = heightmap_position1.y >= heightmap_position2.y ? heightmap_position1.y : heightmap_position2.y;

  auto size = static_cast<int>(terrain->getSize());
  auto left = max(int((heightmap_position1.x - msg->outer_radius) * size), 0);
  auto top = max(int((y_top - msg->outer_radius) * size), 0);
  auto right = min(int((heightmap_position1.x + msg->outer_radius) * size), size);
  auto bottom = min(int((y_bot + msg->outer_radius) * size), size);

  // Render into a offscreen image first
  auto image = cv::Mat(bottom - top + 1, right - left + 1, CV_32FC1);

  for (auto y = top; y <= bottom; ++y)
    for (auto x = left; x <= right; ++x)
    {
      auto ts_x_dist = x / static_cast<double>(size) - heightmap_position1.x;

      auto ts_y_dist_top = y / static_cast<double>(size) - y_top;
      auto ts_y_dist_bot = y / static_cast<double>(size) - y_bot;

      auto ts_y_dist = 0.0f;
      if (ts_y_dist_top < 0)
        ts_y_dist = ts_y_dist_top;
      else if (ts_y_dist_bot > 0)
        ts_y_dist = ts_y_dist_bot;
        
      auto dist = sqrt(ts_y_dist * ts_y_dist + ts_x_dist * ts_x_dist);

      auto inner_weight = 1.0;
      if (dist > msg->inner_radius)
      {
        inner_weight = clamp((dist - msg->inner_radius) / (msg->outer_radius - msg->inner_radius), 0.0, 1.0);
        inner_weight = 1.0 - (inner_weight * inner_weight);
      }

      auto added_height = inner_weight * msg->weight;
      auto new_height = raise_operation ? +added_height : -added_height;
      image.at<float>(y - top, x - left) = new_height;
    }

  applyImageToHeightmap(heightmap, left, top, image, get_height_value, set_height_value);

  gzlog << "DynamicTerrain: capsule " << msg->operation << " operation between ("
    << msg->position1.x << ", " << msg->position1.y << ") and ("
    << msg->position2.x << ", " << msg->position2.y << ")" << endl;
}

void TerrainModifier::modifyPatch(Heightmap* heightmap, const modify_terrain_patch::ConstPtr& msg,
                                  function<float(long, long)> get_height_value,
                                  function<void(long, long, float)> set_height_value)
{
  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return;
  }

  if (msg->patch.encoding != "32FC1")
  {
    gzerr << "DynamicTerrain: Only 32FC1 formats are supported" << endl;
    return;
  }

  auto image_handle = TerrainModifier::importImageToOpenCV(msg);
  if (image_handle == nullptr)
  {
    gzerr << "DynamicTerrain: Failed to convert ROS image" << endl;
    return;
  }

  auto _terrain_position = Vector3(msg->position.x, msg->position.y, 0);
  auto heightmap_position = Vector3();
  terrain->getTerrainPosition(_terrain_position, &heightmap_position);
  auto size = terrain->getSize();
  auto center = Point2i(lroundf(heightmap_position.x * size),lroundf(heightmap_position.y * size));

  applyImageToHeightmap(heightmap, center, image_handle->image, get_height_value, set_height_value);

  gzlog << "DynamicTerrain: patch applied at (" << msg->position.x << ", " << msg->position.y << ")" << endl;
}

CvImageConstPtr TerrainModifier::importImageToOpenCV(const modify_terrain_patch::ConstPtr& msg)
{
  CvImageConstPtr image_handle;

  try
  {
    image_handle = toCvShare(msg->patch, msg, image_encodings::TYPE_32FC1);  // Using single precision (32-bit) float
                                                                             // same as the heightmap
  }
  catch (cv_bridge::Exception& e)
  {
    gzerr << "DynamicTerrain: cv_bridge exception: %s" << e.what() << endl;
    return nullptr;
  }

  return image_handle;
}

void TerrainModifier::applyImageToHeightmap(Heightmap* heightmap, const Point2i& center, const Mat& image,
                          std::function<float(long, long)> get_height_value,
                          std::function<void(long, long, float)> set_height_value)
{
  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return;
  }

  if (image.type() != CV_32FC1)
  {
    gzerr << "DynamicTerrain: Only 32FC1 formats are supported" << endl;
    return;
  }

  auto heightmap_size = static_cast<int>(terrain->getSize());
  auto left = max(center.x - image.cols / 2, 0);
  auto top = max(center.y - image.rows / 2, 0);
  auto right = min(center.x - image.cols / 2 + image.cols - 1, heightmap_size);
  auto bottom = min(center.y - image.rows / 2 + image.rows - 1, heightmap_size);

  for (auto y = top; y <= bottom; ++y)
    for (auto x = left; x <= right; ++x)
    {
      auto row = y - top;
      auto col = x - left;
      auto diff_height = image.at<float>(row, col);
      auto new_height = get_height_value(x, y) + diff_height;
      set_height_value(x, y, new_height);
    }
}