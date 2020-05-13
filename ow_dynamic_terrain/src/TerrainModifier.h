#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include "ow_dynamic_terrain/modify_terrain_circle.h"
#include "ow_dynamic_terrain/modify_terrain_patch.h"

class TerrainModifier
{
public:
  static void modify(gazebo::rendering::Heightmap* heightmap,
                     const ow_dynamic_terrain::modify_terrain_circle::ConstPtr& msg,
                     std::function<float(long, long)> get_height_value,
                     std::function<void(long, long, float)> set_height_value);

public:
  static void modify(gazebo::rendering::Heightmap* heightmap,
                     const ow_dynamic_terrain::modify_terrain_patch::ConstPtr& msg,
                     std::function<float(long, long)> get_height_value,
                     std::function<void(long, long, float)> set_height_value);
};