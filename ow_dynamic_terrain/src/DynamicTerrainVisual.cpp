// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "DynamicTerrainBase.h"
#include "TerrainModifier.h"

using namespace std;
using namespace gazebo;
using namespace rendering;

namespace ow_dynamic_terrain
{
class DynamicTerrainVisual : public VisualPlugin, public DynamicTerrainBase
{
public:
  DynamicTerrainVisual() : DynamicTerrainBase{ "ow_dynamic_terrain", "DynamicTerrainVisual" }
  {
  }

  void Load(VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/) override
  {
    Initialize("visual");
  }

private:
  static inline float getHeightInWorldCoords(const Ogre::Terrain* terrain, int x, int y)
  {
    auto value = terrain->getHeightAtPoint(x, y);
    value += terrain->getPosition().z;
    return value;
  }

  static inline void setHeightFromWorldCoords(Ogre::Terrain* terrain, int x, int y, float value)
  {
    value -= terrain->getPosition().z;
    terrain->setHeightAtPoint(x, y, value);
  }

  template <typename T, typename M>
  void onModifyTerrainMsg(T msg, M modify_method)
  {
    auto heightmap = getHeightmap(get_scene());
    if (heightmap == nullptr)
    {
      gzerr << m_plugin_name << ": Couldn't acquire heightmap!" << endl;
      return;
    }

    modified_terrain_diff diff_msg;

    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
    auto changed = modify_method(heightmap, msg, 
        [&terrain](int x, int y) { return getHeightInWorldCoords(terrain, x, y); },
        [&terrain](int x, int y, float value) { setHeightFromWorldCoords(terrain, x, y, value); }, 
        diff_msg);

    if (changed)
    {
      terrain->updateGeometry();
      terrain->updateDerivedData(false, Ogre::Terrain::DERIVED_DATA_NORMALS | Ogre::Terrain::DERIVED_DATA_LIGHTMAP);

      m_differential_pub.publish(diff_msg);
    }
  }

  void onModifyTerrainCircleMsg(const modify_terrain_circle::ConstPtr& msg) override
  {
    onModifyTerrainMsg(msg, TerrainModifier::modifyCircle);
  }

  void onModifyTerrainEllipseMsg(const modify_terrain_ellipse::ConstPtr& msg) override
  {
    onModifyTerrainMsg(msg, TerrainModifier::modifyEllipse);
  }

  void onModifyTerrainPatchMsg(const modify_terrain_patch::ConstPtr& msg) override
  {
    onModifyTerrainMsg(msg, TerrainModifier::modifyPatch);
  }
};

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrainVisual)
}  // namespace ow_dynamic_terrain
