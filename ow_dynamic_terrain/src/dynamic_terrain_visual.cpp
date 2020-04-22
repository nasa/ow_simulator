#include <gazebo/common/common.hh>

#include "terrain_modifier.h"
#include "shared_constants.h"

using namespace gazebo;

class DynamicTerrainVisual : public VisualPlugin
{
public:
    void Load(rendering::VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/)
    {
        gzlog << "DynamicTerrainVisual: successfully loaded!" << std::endl;

        this->on_update_connection_ = event::Events::ConnectPostRender(
            std::bind(&DynamicTerrainVisual::onUpdate, this));

        hole_drilled_ = false;
        plugin_load_time_ = gazebo::common::Time::GetWallTime();
    }

private:

    rendering::Heightmap* getHeightmap()
    {
        auto scene = rendering::get_scene();
        if (!scene)
        {
            gzerr << "DynamicTerrainVisual: Couldn't acquire scene!" << std::endl;
            return nullptr;
        }

        auto heightmap = scene->GetHeightmap();
        if (heightmap == nullptr)
        {
            gzerr << "DynamicTerrainVisual: scene has no heightmap!" << std::endl;
            return nullptr;
        }

        return heightmap;
    }

    void drillTerrainAt(double x, double y)
    {
        auto heightmap = getHeightmap();
        if (heightmap == nullptr)
        {
            gzerr << "DynamicTerrainVisual: Couldn't acquire heightmap!" << std::endl;
            return;
        }

        auto position_xy = Ogre::Vector3(x, y, 0);
        
        auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
        TerrainModifier::modify(heightmap, "lower", position_xy, 0.003, 0.002, 1.0,
            [&terrain](long x, long y) { return terrain->getHeightAtPoint(x, y); },
            [&terrain](long x, long y, float value) { terrain->setHeightAtPoint(x, y, value); }
        );

        terrain->updateGeometry();
        terrain->updateDerivedData(false,
            Ogre::Terrain::DERIVED_DATA_NORMALS | Ogre::Terrain::DERIVED_DATA_LIGHTMAP);

        hole_drilled_ = true;
        gzlog << "DynamicTerrainVisual: A hole has been drilled at ("
            << position_xy.x << ", " << position_xy.y << ")" << std::endl;
    }

    void onUpdate()
    {
        if (gazebo::common::Time::GetWallTime().sec - plugin_load_time_.sec < SharedConstants::WARM_UP_PERIOD_IN_SECONDS)
            return;

        if (hole_drilled_ || gazebo::common::Time::GetWallTime().sec % 10 != 0)
            return;

        drillTerrainAt(SharedConstants::DRILL_POINT_X, SharedConstants::DRILL_POINT_Y);
    }

private: event::ConnectionPtr on_update_connection_;
private: bool hole_drilled_;                // TODO: remove
private: common::Time plugin_load_time_;    // TODO: remove
};

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrainVisual)