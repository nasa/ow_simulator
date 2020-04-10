
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <OgreVector3.h>

class ModifyTerrain
{
public:
    static void modify(gazebo::rendering::Heightmap* heightmap,
        Ogre::Vector3 terrain_position,
        double outside_radius, double inside_radius, double weight,
        const std::string& op,
        std::function<float (long, long)> get_height_value,
        std::function<void (long, long, float)> set_height_value);
};