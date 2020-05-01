
#include <geometry_msgs/Point.h>
#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>

class TerrainModifier
{
public:
  static void modify(gazebo::rendering::Heightmap* heightmap,
                     const std::string& operation,
                     const geometry_msgs::Point& terrain_position,
                     double outside_radius, double inside_radius,
                     double weight, std::function<float(long, long)> get_height_value,
                     std::function<void(long, long, float)> set_height_value);
};