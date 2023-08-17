// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>
#include <stdexcept>
#include <functional>

#include <MaterialDistributionPlugin.h>

#include <point_cloud_util.h>

#include <populate_materials.h>

using std::string, std::make_unique, std::endl, std::uint8_t;

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

  const auto corner_a = sdf->Get<GridPositionType>(PARAMETER_CORNER_A);
  const auto corner_b = sdf->Get<GridPositionType>(PARAMETER_CORNER_B);
  const auto cell_side_length = sdf->Get<double>(PARAMETER_CELL_SIDE_LENGTH);

  try {
    m_grid = make_unique<AxisAlignedGrid<MaterialBlend>>(corner_a, corner_b,
                                                         cell_side_length);
  } catch (const GridConfigError &e) {
    gzerr << e.what() << endl;
    return;
  }

  const auto center = m_grid->getCenter();
  const auto diagonal = m_grid->getDiagonal();
  gzlog << PLUGIN_NAME
        << ": Material grid centered at (" << center.X() << ", "
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

  m_visual_integrator = make_unique<MaterialIntegrator>(
    m_node_handle.get(), m_grid.get(),
    "/ow_dynamic_terrain/modification_differential/visual",
    "/ow_materials/dug_points2",
    std::bind(&MaterialDistributionPlugin::handleVisualBulk, this,
      std::placeholders::_1),
    std::bind(&MaterialDistributionPlugin::interpolateColor, this,
      std::placeholders::_1)
  );
  m_collision_integrator = make_unique<MaterialIntegrator>(
    m_node_handle.get(), m_grid.get(),
    "/ow_dynamic_terrain/modification_differential/collision",
    "/ow_materials/dug_points2",
    std::bind(&MaterialDistributionPlugin::handleCollisionBulk, this,
      std::placeholders::_1),
    std::bind(&MaterialDistributionPlugin::interpolateColor, this,
      std::placeholders::_1)
  );

  // publish latched, because points will never change
  m_grid_pub = m_node_handle->advertise<sensor_msgs::PointCloud2>(
    "/ow_materials/grid_points2", 1, true);
  // the process takes about 6 seconds, so we spin it off in its own thread to
  // not hold up loading the rest of Gazebo
  std::thread t(&MaterialDistributionPlugin::publishGrid, this);
  t.detach();

  gzlog << PLUGIN_NAME << ": Successfully loaded!" << endl;

}

void MaterialDistributionPlugin::publishGrid()
{
  pcl::PointCloud<pcl::PointXYZRGB> grid_points;
  // publish and latch grid data as a point cloud for visualization
  m_grid->runForEach(
    [&grid_points, this]
    (MaterialBlend b, GridPositionType center) {
      const Color c = interpolateColor(b);
      grid_points.emplace_back(
        static_cast<uint8_t>(c.r), // truncates double to int (<256)
        static_cast<uint8_t>(c.g),
        static_cast<uint8_t>(c.b)
      );

      // WORKAROUND for OW-1194, TF has an incorrect transform for
      //            base_link (specific for atacama_y1a)
      center -= GridPositionType(-1.0, 0.0, 0.37);

      grid_points.back().x = static_cast<float>(center.X());
      grid_points.back().y = static_cast<float>(center.Y());
      grid_points.back().z = static_cast<float>(center.Z());
    }
  );

  publishPointCloud(&m_grid_pub, grid_points);

  gzlog << PLUGIN_NAME << "Grid visualization now ready for viewing in Rviz"
        << std::endl;
}

void MaterialDistributionPlugin::handleVisualBulk(MaterialBlend const &blend)
{
  std::stringstream s;
  s << "handleVisualBulk: STUBBED\n"
       "the bulk contains\n";
  for (auto const &x : blend.getBlendMap())
    s << "\t" << x.second << "%% of " << static_cast<int>(x.first) << "\n";
  gzlog << s.str() << endl;
}

void MaterialDistributionPlugin::handleCollisionBulk(MaterialBlend const &blend)
{
  std::stringstream s;
  s << "handleCollisionBulk: STUBBED\n"
       "the bulk contains\n";
  for (auto const &x : blend.getBlendMap())
    s << "\t" << x.second * 100 << "% of " << static_cast<int>(x.first) << "\n";
  gzlog << s.str() << endl;
}

Color MaterialDistributionPlugin::interpolateColor(
  MaterialBlend const &blend) const
{
  return std::accumulate(blend.getBlendMap().begin(), blend.getBlendMap().end(),
    Color({0.0, 0.0, 0.0}),
    [this]
    (Color value, MaterialBlend::BlendType::value_type const &p) {
      const auto m = m_material_db->getMaterial(p.first);
      // TODO: define + and * operators for color
      return Color(
        {
          value.r + p.second * m.color.r,
          value.g + p.second * m.color.g,
          value.b + p.second * m.color.b
        }
      );
    }
  );
}
