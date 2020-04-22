
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <OgreVector3.h>

class TerrainModifier
{
public:
    static void modify(gazebo::rendering::Heightmap* heightmap,
        const std::string& op,
        Ogre::Vector3 terrain_position,
        double outside_radius, double inside_radius, double weight,
        std::function<float (long, long)> get_height_value,
        std::function<void (long, long, float)> set_height_value);
};