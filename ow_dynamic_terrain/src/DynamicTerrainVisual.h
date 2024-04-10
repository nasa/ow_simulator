// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "DynamicTerrainBase.h"
#include "TerrainModifier.h"
#include <gazebo/rendering/RenderingIface.hh>

namespace ow_dynamic_terrain
{
class DynamicTerrainVisual : public gazebo::VisualPlugin, public DynamicTerrainBase
{
public:
  DynamicTerrainVisual();

  void Load(gazebo::rendering::VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/) override;

private:
  static inline float getHeightInWorldCoords(const Ogre::Terrain* terrain, int x, int y);

  static inline void setHeightFromWorldCoords(Ogre::Terrain* terrain, int x, int y, float value);

  template <typename T, typename M>
  void onModifyTerrainMsg(T msg, M modify_method)
  {
    // NOTE: the first modification message starts at seq=1, not sure why
    static std::uint32_t next_expected_seq = 1;
    if (msg->header.seq != next_expected_seq) {
      ROS_WARN_STREAM("Visual modification message was dropped! At least "
                      << (msg->header.seq - next_expected_seq)
                      << " message(s) may have been missed.");
    }
    next_expected_seq = msg->header.seq + 1;

    auto heightmap = getHeightmap(gazebo::rendering::get_scene());
    if (heightmap == nullptr)
    {
      gzerr << m_plugin_name << ": DynamicTerrainVisual::onModifyTerrainMsg()"
        " could not acquire heightmap." << std::endl;
      return;
    }

    modified_terrain_diff diff_msg;

    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
    auto changed = modify_method(heightmap, mMaterialMaskTexture, msg,
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

  void onModifyTerrainCircleMsg(const modify_terrain_circle::ConstPtr& msg) override;

  void onModifyTerrainEllipseMsg(const modify_terrain_ellipse::ConstPtr& msg) override;

  void onModifyTerrainPatchMsg(const modify_terrain_patch::ConstPtr& msg) override;

  void initMaskTexture(unsigned int texSize);

  void initTextureUnits();

  gazebo::event::ConnectionPtr m_on_prerender_connection;

  Ogre::TexturePtr mMaterialMaskTexture;
};

}  // namespace ow_dynamic_terrain
