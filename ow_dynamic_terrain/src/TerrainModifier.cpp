// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <OgreVector3.h>
#include <sensor_msgs/image_encodings.h>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include "MergeMethods.h"
#include "OpenCV_Util.h"
#include "TerrainBrush.h"
#include "TerrainModifier.h"

using namespace std;
using namespace Ogre;
using namespace gazebo;
using namespace rendering;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;
using namespace ow_dynamic_terrain;

static void formatDiffMsg(const CvImage& diff_image, const Point32& position,
                          float scale_factor, int rows, int cols, string op_name,
                          ow_dynamic_terrain::modified_terrain_diff& out_diff_msg)
{
  diff_image.toImageMsg(out_diff_msg.diff);
  out_diff_msg.position = position;
  out_diff_msg.height = rows / scale_factor;
  out_diff_msg.width  = cols / scale_factor;

  gzlog << "DynamicTerrain: " << op_name << " operation performed at (" 
        << position.x << ", " << position.y << ")"
        << endl;
}

bool TerrainModifier::modifyCircle(Heightmap* heightmap, const modify_terrain_circle::ConstPtr& msg,
                                   const function<float(int, int)>& get_height_value,
                                   const function<void(int, int, float)>& set_height_value,
                                   ow_dynamic_terrain::modified_terrain_diff& out_diff_msg)
{
  GZ_ASSERT(heightmap != nullptr, "heightmap is null!");

  if (msg->outer_radius <= 0.0f)
  {
    gzerr << "DynamicTerrain: outer_radius has to be a positive number!" << endl;
    return false;
  }

  if (msg->inner_radius > msg->outer_radius)
  {
    gzerr << "DynamicTerrain: inner_radius can't exceed outer_radius value!" << endl;
    return false;
  }

  auto merge_method = MergeMethods::mergeMethodFromString(msg->merge_method != "" ? msg->merge_method : "add");
  if (!merge_method)
  {
    gzerr << "DynamicTerrain: merge method [" << msg->merge_method << "] is unsupported!" << endl;
    return false;
  }

  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return false;
  }

  auto center = TerrainModifier::getHeightmapPosition(heightmap, msg->position);
  auto h_scale = terrain->getSize() / terrain->getWorldSize();  // horizontal scale factor
  auto image = TerrainBrush::circle(h_scale * msg->outer_radius, h_scale * msg->inner_radius, msg->weight);

  CvImage differential_image;
  auto changed = applyImageToHeightmap(heightmap, center, msg->position.z, image, false, 
                                       get_height_value, set_height_value, *merge_method,
                                       differential_image);

  if (changed)
    formatDiffMsg(differential_image, msg->position, h_scale,
                  image.rows, image.cols, "circle", out_diff_msg);

  return changed;
}

bool TerrainModifier::modifyEllipse(Heightmap* heightmap, const modify_terrain_ellipse::ConstPtr& msg,
                                    const function<float(int, int)>& get_height_value,
                                    const function<void(int, int, float)>& set_height_value,
                                    ow_dynamic_terrain::modified_terrain_diff& out_diff_msg)
{
  GZ_ASSERT(heightmap != nullptr, "heightmap is null!");

  if (msg->outer_radius_a <= 0.0f || msg->outer_radius_b <= 0.0f)
  {
    gzerr << "DynamicTerrain: outer_radius a & b has to be positive!" << endl;
    return false;
  }

  if (msg->inner_radius_a > msg->outer_radius_a || msg->inner_radius_b > msg->outer_radius_b)
  {
    gzerr << "DynamicTerrain: inner_radius can't exceed outer_radius value!" << endl;
    return false;
  }

  auto merge_method = MergeMethods::mergeMethodFromString(msg->merge_method != "" ? msg->merge_method : "add");
  if (!merge_method)
  {
    gzerr << "DynamicTerrain: merge method [" << msg->merge_method << "] is unsupported!" << endl;
    return false;
  }

  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return false;
  }

  auto center = TerrainModifier::getHeightmapPosition(heightmap, msg->position);

  auto h_scale = terrain->getSize() / terrain->getWorldSize();  // horizontal scale factor
  auto image =
      TerrainBrush::ellipse(h_scale * msg->outer_radius_a, h_scale * msg->inner_radius_a,
                            h_scale * msg->outer_radius_b, h_scale * msg->inner_radius_b, msg->weight);

  if (!ignition::math::equal<float>(msg->orientation, 0.0f))  // Avoid performing the rotation if orientation is zero
  {
    image = OpenCV_Util::expandImage(image);  // expand the image to hold rotation output with no loss
    image = OpenCV_Util::rotateImage(image, msg->orientation);
  }

  cv_bridge::CvImage differential_image;
  auto changed = applyImageToHeightmap(heightmap, center, msg->position.z, image, false, 
                                       get_height_value, set_height_value, *merge_method,
                                       differential_image);

  if (changed)
    formatDiffMsg(differential_image, msg->position, h_scale,
                  image.rows, image.cols, "ellipse", out_diff_msg);

  return changed;
}

bool TerrainModifier::modifyPatch(Heightmap* heightmap, const modify_terrain_patch::ConstPtr& msg,
                                  const function<float(int, int)>& get_height_value,
                                  const function<void(int, int, float)>& set_height_value,
                                  ow_dynamic_terrain::modified_terrain_diff& out_diff_msg)
{
  GZ_ASSERT(heightmap != nullptr, "heightmap is null!");

  auto merge_method = MergeMethods::mergeMethodFromString(msg->merge_method != "" ? msg->merge_method : "add");
  if (!merge_method)
  {
    gzerr << "DynamicTerrain: merge method [" << msg->merge_method << "] is unsupported!" << endl;
    return false;
  }

  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
    return false;
  }

  if (msg->patch.encoding != "32FC1")
  {
    gzerr << "DynamicTerrain: Only 32FC1 formats are supported" << endl;
    return false;
  }

  auto center = TerrainModifier::getHeightmapPosition(heightmap, msg->position);
  auto h_scale = terrain->getSize() / terrain->getWorldSize();  // horizontal scale factor

  auto image_handle = TerrainModifier::importImageToOpenCV(msg);
  if (image_handle == nullptr)
  {
    gzerr << "DynamicTerrain: Failed to convert ROS image" << endl;
    return false;
  }

  auto image = image_handle->image;
  if (!ignition::math::equal<float>(msg->orientation, 0.0f))  // Avoid performing the rotation if orientation is zero
  {
    image = OpenCV_Util::expandImage(image);  // expand the image to hold rotation output with no loss
    image = OpenCV_Util::rotateImage(image, msg->orientation);
  }

  cv_bridge::CvImage differential_image;
  auto changed = applyImageToHeightmap(heightmap, center, msg->position.z, image, false, 
                                       get_height_value, set_height_value, *merge_method,
                                       differential_image);

  if (changed)
    formatDiffMsg(differential_image, msg->position, h_scale,
                  image.rows, image.cols, "patch", out_diff_msg);

  return changed;
}

Point2i TerrainModifier::getHeightmapPosition(Heightmap* heightmap, const geometry_msgs::Point32& position)
{
  auto _terrain_position = Vector3(position.x, position.y, 0);
  auto heightmap_position = Vector3();
  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
  terrain->getTerrainPosition(_terrain_position, &heightmap_position);
  auto heightmap_size = terrain->getSize();
  return Point2i(lroundf(heightmap_size * heightmap_position.x), lroundf(heightmap_size * heightmap_position.y));
}

CvImageConstPtr TerrainModifier::importImageToOpenCV(const modify_terrain_patch::ConstPtr& msg)
{
  auto image_handle = CvImageConstPtr();

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

bool TerrainModifier::applyImageToHeightmap(Heightmap* heightmap, const Point2i& center, float z_bias,
                                            const Mat& image, bool skip_zeros,
                                            const function<float(int, int)>& get_height_value,
                                            const function<void(int, int, float)>& set_height_value,
                                            const function<float(float, float)>& merge_method, 
                                            cv_bridge::CvImage& out_diff_image)
{
  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  if (image.type() != CV_32FC1)
  {
    gzerr << "DynamicTerrain: Only 32FC1 formats are supported" << endl;
    return false;
  }

  auto heightmap_size = static_cast<int>(terrain->getSize());
  auto left = max(center.x - image.cols / 2, 0);
  auto top = max(center.y - image.rows / 2, 0);
  auto right = min(center.x - image.cols / 2 + image.cols - 1, heightmap_size);
  auto bottom = min(center.y - image.rows / 2 + image.rows - 1, heightmap_size);

  auto diff = OpenCV_Util::createZerosMatLike(image);

  bool change_occurred = false;

  for (auto y = top; y <= bottom; ++y)
    for (auto x = left; x <= right; ++x)
    {
      auto pixel_value = image.at<float>(y - top, x - left);
      if (skip_zeros && ignition::math::equal<float>(pixel_value, 0.0f))
        continue;
      
      auto old_height = get_height_value(x, y);
      auto new_height = merge_method(old_height, pixel_value + z_bias);

      if (old_height == new_height) 
        continue; // no change is necessary
      
      set_height_value(x, y, new_height);
      diff.at<float>(y - top, x - left) = new_height - old_height;
      
      // if we make it here, flag that a change has occurred
      change_occurred = true;
    }
  
    out_diff_image.image    = diff;
    out_diff_image.encoding = image_encodings::TYPE_32FC1;
  
    return change_occurred;
}
