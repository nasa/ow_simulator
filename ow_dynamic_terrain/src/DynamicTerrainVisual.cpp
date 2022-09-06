// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "DynamicTerrainBase.h"
#include "TerrainModifier.h"
#include <gazebo/rendering/RenderingIface.hh>

using namespace std;
using namespace gazebo;
using namespace rendering;
using namespace Ogre;

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

    m_on_prerender_connection = gazebo::event::Events::ConnectPreRender([this]() {
      // Initialize terrain with Ogre-generated normal map.
      applyNormalMap();
      // Disconnect so this method will not be called again.
      m_on_prerender_connection.reset();
    });
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
      gzerr << m_plugin_name << ": DynamicTerrainVisual::onModifyTerrainMsg()"
        " could not acquire heightmap." << endl;
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

  void applyNormalMap()
  {
    auto heightmap = getHeightmap(get_scene());
    if (heightmap == nullptr)
    {
      gzerr << m_plugin_name << ": DynamicTerrainVisual::applyNormalMap()"
        " could not acquire heightmap." << endl;
      return;
    }
    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

    // Override custom material normal map labeled "ow_dynamic_terrain_normal_map"
    // with Ogre::Terrain's internal normal map.
    // Ogre::Terrain generates an internal normal map and it is updated by
    // Terrain::updateDerivedData(). But if you use a custom material script
    // with a custom normal map, the internal normal map will never be seen.
    // Note: If you are not using a custom material this code will have no
    // effect, but Ogre should be using the correct normal map and a generated
    // material and shader in that case.
    // Caveat: The internal normal map is always sized to the terrain's DEM
    // resolution, so using a larger texture for more detail is not possible
    // with this plugin.
    ResourceManager::ResourceMapIterator res_it = MaterialManager::getSingleton().getResourceIterator();
    while (res_it.hasMoreElements())
    {
      ResourcePtr resource = res_it.getNext();
      MaterialPtr material = resource.staticCast<Material>();
      Material::TechniqueIterator tech_it = material->getTechniqueIterator();
      while (tech_it.hasMoreElements())
      {
        Technique* technique = tech_it.getNext();
        Technique::PassIterator pass_it = technique->getPassIterator();
        while (pass_it.hasMoreElements())
        {
          Pass* pass = pass_it.getNext();
          TextureUnitState* tus = pass->getTextureUnitState("ow_dynamic_terrain_normal_map");
          if (tus != 0)
          {
            tus->setTexture(terrain->getTerrainNormalMap());
          }
        }
      }
    }
  }

  gazebo::event::ConnectionPtr m_on_prerender_connection;
};

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrainVisual)
}  // namespace ow_dynamic_terrain
