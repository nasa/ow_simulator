// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>
#include <stdexcept>
#include <functional>

#include <MaterialDistributionPlugin.h>

#include <populate_materials.h>

using std::string, std::make_unique, std::endl;
using ignition::math::Vector3d;

using namespace ow_materials;
using namespace gazebo;

const string PLUGIN_NAME = "MaterialDistributionPlugin";

const string NAMESPACE_MATERIALS = "/ow_materials";

const string NODE_NAMES = "/ow_materials/material_distribution_plugin";

const string PARAMETER_CORNER_A         = "corner_a";
const string PARAMETER_CORNER_B         = "corner_b";
const string PARAMETER_CELL_SIDE_LENGTH = "cell_side_length";

void MaterialDistributionPlugin::Load(physics::ModelPtr model,
                                      sdf::ElementPtr sdf)
{
  m_node_handle = make_unique<ros::NodeHandle>(NODE_NAMES);

  if (!sdf->HasElement(PARAMETER_CORNER_A)
      || !sdf->HasElement(PARAMETER_CORNER_B)) {
    gzerr << "Both " << PARAMETER_CORNER_A << " and " << PARAMETER_CORNER_B
          << " parameters are required." << endl;
    return;
  }
  if (!sdf->HasElement(PARAMETER_CELL_SIDE_LENGTH)) {
    gzerr << PARAMETER_CELL_SIDE_LENGTH << " is required." << endl;
    return;
  }

  const auto corner_a = sdf->Get<Vector3d>(PARAMETER_CORNER_A);
  const auto corner_b = sdf->Get<Vector3d>(PARAMETER_CORNER_B);
  const auto cell_side_length = sdf->Get<double>(PARAMETER_CELL_SIDE_LENGTH);

  auto base_blend = MaterialBlend();
  base_blend.m_blend = {{0, 0.6}, {2, 0.4}};
  try {
    m_grid = make_unique<AxisAlignedGrid<MaterialBlend>>(
      corner_a.X(), corner_a.Y(), corner_a.Z(),
      corner_b.X(), corner_b.Y(), corner_b.Z(),
      cell_side_length, base_blend
    );
  } catch (const GridConfigError &e) {
    gzerr << e.what() << endl;
    return;
  }

  const auto center = m_grid->getCenter();
  const auto diagonal = m_grid->getDiagonal();
  gzlog << PLUGIN_NAME << ": Material grid centered at (" << center.X() << ", "
                                                          << center.Y() << ", "
                                                          << center.Z() << ") "
        << "with dimensions " << diagonal.X() << " x "
                              << diagonal.Y() << " x "
                              << diagonal.Z() << " meters.\n";

  m_material_db = make_unique<MaterialDatabase>();
  // populate materials database
  try {
    populate_material_database(m_material_db.get(), NAMESPACE_MATERIALS);
  } catch (const MaterialConfigError &e) {
    gzerr << e.what() << endl;
    return;
  }
  gzlog << PLUGIN_NAME << ": Materials database populated with "
        << m_material_db->size() << " materials.\n";

  m_visual_integrator = make_unique<MaterialIntegrator>(m_node_handle.get(),
    "/ow_dynamic_terrain/modification_differential/visual", m_grid.get(),
    std::bind(&MaterialDistributionPlugin::handleVisualBulk, this,
      std::placeholders::_1),
    std::bind(&MaterialDistributionPlugin::interpolateColor, this,
      std::placeholders::_1)
  );
  m_collision_integrator = make_unique<MaterialIntegrator>(m_node_handle.get(),
    "/ow_dynamic_terrain/modification_differential/collision", m_grid.get(),
    std::bind(&MaterialDistributionPlugin::handleCollisionBulk, this,
      std::placeholders::_1),
    std::bind(&MaterialDistributionPlugin::interpolateColor, this,
      std::placeholders::_1)
  );

  gzlog << PLUGIN_NAME << ": Successfully loaded!" << endl;

  // DEBUG
  // auto a = m_grid->getCenter();
  // gzlog << "a = " << a.X() << " x "
  //                   << a.Y() << " x "
  //                   << a.Z() << " meters.\n";
  // if (m_grid->containsPoint(a.X(), a.Y(), a.Z())) {
  //   auto blend_000 = m_grid->getCellValueAtPoint(a.X(), a.Y(), a.Z());
  //   gzlog << PLUGIN_NAME << "0, 0, 0 cell contents:";
  //   for (const auto &p : blend_000.m_blend) {
  //     gzlog << "  " << static_cast<int>(p.first) << ": " << p.second;
  //   }
  //   gzlog << endl;
  // } else {
  //   gzlog << "DOES NOT CONTAIN" << endl;
  // }


  // if (m_grid->containsPoint(a.X(), a.Y(), a.Z())) {
  //   auto blend_000 = m_grid->getCellValueAtPoint(a.X(), a.Y(), a.Z());
  //   gzlog << PLUGIN_NAME << "0, 0, 0 cell contents:";
  //   for (const auto &p : blend_000.m_blend) {
  //     gzlog << "  " << p.first << ": " << p.second;
  //   }
  //   gzlog << endl;
  // } else {
  //   gzerr << PLUGIN_NAME << "DOES NOT CONTAIN POINT" << endl;
  // }


}

// DEBUG
#include <sstream>
#include <gazebo/gazebo.hh>

void MaterialDistributionPlugin::handleVisualBulk(MaterialBlend const &blend)
{
  std::stringstream s;
  s << "handleVisualBulk: the bulk contains\n";
  for (auto const &x : blend.m_blend)
    s << "\t" << x.second << "%% of " << static_cast<int>(x.first) << "\n";
  gzlog << s.str() << endl;

  

  // TODO: publish points2 topic with bulk color

}

void MaterialDistributionPlugin::handleCollisionBulk(MaterialBlend const &blend)
{
  std::stringstream s;
  s << "handleCollisionBulk: the bulk contains\n";
  for (auto const &x : blend.m_blend)
    s << "\t" << x.second << "%% of " << static_cast<float>(x.first) << "\n";
  gzlog << s.str() << endl;
}

Color MaterialDistributionPlugin::interpolateColor(MaterialBlend const &blend) const
{
  Color c = std::accumulate(blend.m_blend.begin(), blend.m_blend.end(),
    Color({0.0, 0.0, 0.0}),
    [this]
    (Color value, MaterialBlend::BlendType::value_type const &p) {
      const auto m = m_material_db->getMaterial(p.first);
      return Color(
        {
          value.r + p.second * m.color.r,
          value.g + p.second * m.color.g,
          value.b + p.second * m.color.b
        }
      );
    }
  );
  const auto s = blend.m_blend.size();
  return Color({c.r / s, c.g / s, c.b / s});
}
