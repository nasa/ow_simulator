// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef TERRAIN_MODIFIER_H
#define TERRAIN_MODIFIER_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <gazebo/rendering/Heightmap.hh>
#include "ow_dynamic_terrain/modify_terrain_circle.h"
#include "ow_dynamic_terrain/modify_terrain_ellipse.h"
#include "ow_dynamic_terrain/modify_terrain_patch.h"
#include "ow_dynamic_terrain/modified_terrain_diff.h"

namespace ow_dynamic_terrain
{
class TerrainModifier
{
public:
  static bool modifyCircle(gazebo::rendering::Heightmap* heightmap,
                           const ow_dynamic_terrain::modify_terrain_circle::ConstPtr& msg,
                           const std::function<float(int, int)>& get_height_value,
                           const std::function<void(int, int, float)>& set_height_value,
                           ow_dynamic_terrain::modified_terrain_diff& out_diff_msg);

  static bool modifyEllipse(gazebo::rendering::Heightmap* heightmap,
                            const ow_dynamic_terrain::modify_terrain_ellipse::ConstPtr& msg,
                            const std::function<float(int, int)>& get_height_value,
                            const std::function<void(int, int, float)>& set_height_value,
                            ow_dynamic_terrain::modified_terrain_diff& out_diff_msg);

  static bool modifyPatch(gazebo::rendering::Heightmap* heightmap,
                          const ow_dynamic_terrain::modify_terrain_patch::ConstPtr& msg,
                          const std::function<float(int, int)>& get_height_value,
                          const std::function<void(int, int, float)>& set_height_value,
                          ow_dynamic_terrain::modified_terrain_diff& out_diff_msg);

private:
  // converts a world position to a heightmap position in heightmap image coordiantes.
  // param heightmap: reference heightmap used for the conversion.
  // param position: a position in world coordinates
  static cv::Point2i getHeightmapPosition(gazebo::rendering::Heightmap* heightmap,
                                          const geometry_msgs::Point32& position);

  // Imports an OpenCV Matrix object from a sensor_msgs::Image object through cv_bridge
  static cv_bridge::CvImageConstPtr importImageToOpenCV(const ow_dynamic_terrain::modify_terrain_patch::ConstPtr& msg);

  // Applies the an OpenCV image to a heightmap at a given position
  // param heightmap: heightmap to merge the image with
  // param center: absolute position within the heightmap where the image will be applied
  // param z_bias: a value that will be applied as an offset to height values retrieved from the image.
  // param image: a 2D matrix containing the height values (given as 32-bit floats) to be applied/merged.
  // param skip_zeros: if true, pixels in image that are equal to zero will be skipped over.
  // param get_height_value: a lambda function to retrive the height value from the heightmap
  // param set_height_value: a lambda function to set back the height value on the heightmap.
  // param merge_method: Choices are keep, replace, add, sub, min, max and avg.
  // param out_diff_image: an image that stores the change in heightmap around the tool
  // return: true if there was a change made to the heightmap, false otherwise
  static bool applyImageToHeightmap(gazebo::rendering::Heightmap* heightmap,
                                    const cv::Point2i& center, float z_bias,
                                    const cv::Mat& image, bool skip_zeros,
                                    const std::function<float(int, int)>& get_height_value,
                                    const std::function<void(int, int, float)>& set_height_value,
                                    const std::function<float(float, float)>& merge_method,
                                    cv_bridge::CvImage& out_diff_image);
};
}  // namespace ow_dynamic_terrain

#endif  // TERRAIN_MODIFIER_H
