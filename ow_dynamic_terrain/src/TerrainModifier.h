#ifndef TERRAIN_MODIFIER_H
#define TERRAIN_MODIFIER_H

#include <cv_bridge/cv_bridge.h>
#include <gazebo/rendering/Heightmap.hh>
#include "ow_dynamic_terrain/modify_terrain_circle.h"
#include "ow_dynamic_terrain/modify_terrain_capsule.h"
#include "ow_dynamic_terrain/modify_terrain_patch.h"

namespace ow_dynamic_terrain
{
class TerrainModifier
{
public:
  static void modifyCircle(gazebo::rendering::Heightmap* heightmap,
                           const ow_dynamic_terrain::modify_terrain_circle::ConstPtr& msg,
                           std::function<float(long, long)> get_height_value,
                           std::function<void(long, long, float)> set_height_value);

public:
  static void modifyCapsule(gazebo::rendering::Heightmap* heightmap,
                           const ow_dynamic_terrain::modify_terrain_capsule::ConstPtr& msg,
                           std::function<float(long, long)> get_height_value,
                           std::function<void(long, long, float)> set_height_value);

public:
  static void modifyPatch(gazebo::rendering::Heightmap* heightmap,
                          const ow_dynamic_terrain::modify_terrain_patch::ConstPtr& msg,
                          std::function<float(long, long)> get_height_value,
                          std::function<void(long, long, float)> set_height_value);

private:
  static cv_bridge::CvImageConstPtr importImageToOpenCV(const ow_dynamic_terrain::modify_terrain_patch::ConstPtr& msg);
};
}  // namespace ow_dynamic_terrain

#endif  // TERRAIN_MODIFIER_H