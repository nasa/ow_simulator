// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <gazebo/physics/physics.hh>
#include "TerrainModifier.h"
#include "DynamicTerrainBase.h"

#if GAZEBO_MAJOR_VERSION < 9 || (GAZEBO_MAJOR_VERSION == 9 && GAZEBO_MINOR_VERSION < 13)
#error "Gazebo 9.13 or higher is required for this module"
#endif

using namespace std;
using namespace gazebo;
using namespace rendering;
using namespace physics;

// TODO (optimization): Instead of performing EnableAllModels, one could probably limit model enabling operation to the
// the area affected by the terrain modify operation. For our purpose, performing EnableAllModels is not going to have
// a pentality on our system as the number of scene objects is very low.

namespace ow_dynamic_terrain
{
class DynamicTerrainModel : public ModelPlugin, public DynamicTerrainBase
{
public:
  DynamicTerrainModel() : DynamicTerrainBase{ "ow_dynamic_terrain", "DynamicTerrainModel" }
  {
  }

  void Load(ModelPtr model, sdf::ElementPtr /*sdf*/) override
  {
    GZ_ASSERT(model != nullptr, "DynamicTerrainModel: model can't be null!");

    m_model = model;

    auto world = model->GetWorld();
    if (world) {
      world->Physics()->SetParam("ode_quiet", true);
      gzerr << m_plugin_name << ": ODE's LCP Error messages have been suppressed!" << endl;
    }

    Initialize("collision");
  }

private:
  HeightmapShapePtr getHeightmapShape()
  {
    auto& links = m_model->GetLinks();

    if (links.size() == 0)
    {
      gzerr << m_plugin_name << ": Associcated model has no links!" << endl;
      return nullptr;
    }

    auto& link0 = links[0];

    const auto& collisions = link0->GetCollisions();

    if (collisions.size() == 0)
    {
      gzerr << m_plugin_name << ": Model has no collisions for first link!" << endl;
      return nullptr;
    }

    auto& collision = collisions[0];
    if (collision == nullptr)
    {
      gzerr << m_plugin_name << ": Couldn't acquire heightmap model collision!" << endl;
      return nullptr;
    }

    auto shape = boost::dynamic_pointer_cast<HeightmapShape>(collision->GetShape());
    if (shape == nullptr)
    {
      gzerr << m_plugin_name << ": Couldn't acquire heightmap model collision!" << endl;
      return nullptr;
    }

    return shape;
  }

  static inline float getHeightInWorldCoords(const HeightmapShapePtr& heightmap_shape, int x, int y)
  {
    auto value = heightmap_shape->GetHeight(x, heightmap_shape->VertexCount().Y() - y - 1);
    value += heightmap_shape->Pos().Z();
    return value;
  }

  static inline void setHeightFromWorldCoords(const HeightmapShapePtr& heightmap_shape, int x, int y, float value)
  {
    value -= heightmap_shape->Pos().Z();
    heightmap_shape->SetHeight(x, heightmap_shape->VertexCount().Y() - y - 1, value);
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

    auto heightmap_shape = getHeightmapShape();
    if (heightmap_shape == nullptr)
    {
      gzerr << m_plugin_name << ": Couldn't acquire heightmap shape!" << endl;
      return;
    }

    modified_terrain_diff diff_msg;

    auto changed = modify_method(heightmap, msg, 
        [&heightmap_shape](int x, int y) { return getHeightInWorldCoords(heightmap_shape, x, y); },
        [&heightmap_shape](int x, int y, float value) { setHeightFromWorldCoords(heightmap_shape, x, y, value); },
        diff_msg);

    if (changed)
    {
      // Re-enable physics updates for models that may have entered a standstill state
      m_model->GetWorld()->EnableAllModels();
      // publish differential
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

private:
  ModelPtr m_model;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DynamicTerrainModel)
}  // namespace ow_dynamic_terrain
