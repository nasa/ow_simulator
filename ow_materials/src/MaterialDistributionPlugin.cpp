// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>
#include <stdexcept>
#include <functional>

#include <point_cloud_util.h>
#include <gazebo/rendering/rendering.hh>

#include <MaterialDistributionPlugin.h>
#include <populate_materials.h>

#include <Eigen/Core>

using std::string, std::make_unique, std::endl, std::uint8_t,
      std::runtime_error, std::make_pair;

using namespace ow_materials;
using namespace gazebo;

const string PLUGIN_NAME = "MaterialDistributionPlugin";

const string NAMESPACE_MATERIALS = "/ow_materials";

const string NODE_NAME = "/ow_materials/material_distribution_plugin";

const string PARAMETER_CORNER_A         = "corner_a";
const string PARAMETER_CORNER_B         = "corner_b";
const string PARAMETER_CELL_SIDE_LENGTH = "cell_side_length";

void MaterialDistributionPlugin::Load(physics::ModelPtr model,
                                      sdf::ElementPtr sdf)
{
  gzlog << PLUGIN_NAME << ": loading..." << endl;

  m_node_handle = make_unique<ros::NodeHandle>(NODE_NAME);

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
  // message is published later on after the pointcloud has been generated
  m_grid_pub = m_node_handle->advertise<sensor_msgs::PointCloud2>(
    "/ow_materials/grid_points2", 1, true);

  // defer acquiring heightmap albedo texture until the gazebo scene exists
  // NOTE: this event callback will only be called as many times as it needs to
  //  before it acquires the texture, then it will delete its own event handle
  //  to prevent future calls
  m_temp_render_connection = make_unique<event::ConnectionPtr>(
    event::Events::ConnectRender(
      std::bind(&MaterialDistributionPlugin::getHeightmapAlbedo, this)
    )
  );

  gzlog << PLUGIN_NAME << ": awaiting scene heightmap..." << endl;
}

void MaterialDistributionPlugin::getHeightmapAlbedo() {
  // try to get scene and heightmap
  // if either are unavailable, try again next render signal
  auto scene = rendering::get_scene();
  if (scene == nullptr) return;
  rendering::Heightmap *heightmap = scene->GetHeightmap();
  if (heightmap == nullptr) return;
  gzlog << PLUGIN_NAME << ": Gazebo heightmap acquired." << endl;

  // a long string of Gazebo and Ogre API calls to get heightmap albedo texture
  Ogre::TexturePtr texture;
  Ogre::Terrain *terrain;
  try {
    Ogre::TerrainGroup *terrain_group = heightmap->OgreTerrain();
    if (terrain_group == nullptr) throw runtime_error("terrain group");
    terrain = terrain_group->getTerrain(0, 0);
    if (terrain == nullptr) throw runtime_error("terrain");
    Ogre::MaterialPtr material = terrain->getMaterial();
    if (material.isNull()) throw runtime_error("material");
    Ogre::Technique *tech = material->getTechnique(0u);
    if (tech == nullptr) throw runtime_error("technique");
    Ogre::Pass *pass = tech->getPass(0u);
    if (pass == nullptr) throw runtime_error("pass");
    const string UNIT_NAME = "albedoMap";
    Ogre::TextureUnitState *unit = pass->getTextureUnitState(UNIT_NAME);
    if (unit == nullptr) throw runtime_error(UNIT_NAME + " texture unit");
    texture = unit->_getTexturePtr();
    if (texture.isNull()) throw runtime_error("texture");
  } catch (runtime_error e) {
    gzerr << "Failed to acquire heightmap albedo texture: "
          << e.what() << " is missing." << endl;
    // disable callback
    delete m_temp_render_connection.release();
    return;
  }
  Ogre::Image albedo;
  texture->convertToImage(albedo);
  gzlog << PLUGIN_NAME << ": Heightmap albedo texture acquired." << endl;
  // the process can take as long as 10 seconds, so we spin it off in its own
  // thread to not hold up loading the rest of Gazebo
  std::thread t(&MaterialDistributionPlugin::populateGrid, this, albedo, terrain);
  t.detach();
  // the callback's sole purpose has been fulfilled, so we can disable it by
  // deleting its event handle
  delete m_temp_render_connection.release();
}

void MaterialDistributionPlugin::populateGrid(Ogre::Image albedo,
                                              Ogre::Terrain *terrain)
{
  gzlog << PLUGIN_NAME << ": Populating grid from heightmap albedo..." << endl;

  // NOTE: restricted to only 3 for now
  auto basis = m_material_db->getColorBasis();
  // pack color basis into 3xN matrix, where N is the number of possible colors
  constexpr size_t COLOR_COUNT = 3;
  std::vector<float> temp((COLOR_COUNT + 1) * basis.size());
  for (size_t i = 0; i != basis.size(); ++i) {
    temp[(COLOR_COUNT + 1) * i]     = basis[i].second.r / 255.0f;
    temp[(COLOR_COUNT + 1) * i + 1] = basis[i].second.g / 255.0f;
    temp[(COLOR_COUNT + 1) * i + 2] = basis[i].second.b / 255.0f;
    temp[(COLOR_COUNT + 1) * i + 3] = 1.0f;
  }
  Eigen::Matrix4Xf A = Eigen::Matrix4Xf::Map(&temp[0],
                                             COLOR_COUNT + 1,
                                             basis.size());
  Eigen::ColPivHouseholderQR<Eigen::Matrix4Xf> dec(A);
  // Eigen::NNLS<Eigen::Matrix4Xf> dec(A);
  // Eigen::JacobiSVD<Eigen::Matrix4Xf> dec(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // this map enables fast z-column-wise modification of the grid
  // TODO: use unordered_map instead; a custom pair hash function is required
  using AlbedoMaterialMap = std::map<std::pair<size_t, size_t>,
                                     std::pair<MaterialBlend, Color>>;
  AlbedoMaterialMap albedo_blends;
  AlbedoMaterialMap::const_iterator last_insert = albedo_blends.begin();
  // point cloud object that will be published after the loop
  pcl::PointCloud<pcl::PointXYZRGB> grid_points;

  m_grid->runForEach(
    [&]
    (MaterialBlend &blend, GridPositionType center) {
      // acquire x and y in terrain space (between 0 and 1)
      auto tpos = terrain->convertPosition(
        Ogre::Terrain::Space::WORLD_SPACE,
        Ogre::Vector3(center.X(), center.Y(), center.Z()),
        Ogre::Terrain::Space::TERRAIN_SPACE
      );
      auto tex_coord = make_pair(
        static_cast<size_t>(tpos.x * albedo.getWidth()),
        // y terrain coordinate must be flipped to work with image coordinates
        static_cast<size_t>((1.0f - tpos.y) * albedo.getHeight())
      );

      auto it = albedo_blends.find(tex_coord);
      if (it == albedo_blends.end()) {
        Color tcolor(albedo.getColourAt(tex_coord.first, tex_coord.second, 0u));
        Eigen::Vector4f b(tcolor.r / 255.0f, tcolor.g / 255.0f, tcolor.b / 255.0f, 1.0f);
        // solve
        Eigen::VectorXf concentrations = dec.solve(b);
        // save resulting concentrations to the material blend for the cell
        const auto coef_sum = concentrations.lpNorm<1>();
        for (size_t i = 0; i != basis.size(); ++i) {
          // normalize concentrations so they all add up to 1.0
          // blend.add(basis[i].first, concentrations[i] / coef_sum);
          blend.add(basis[i].first, concentrations[i]);
        }
        last_insert = albedo_blends.emplace_hint(
          last_insert,
          make_pair(
            tex_coord,
            make_pair(blend, interpolateColor(blend))
          )
        );
        gzlog << "L1 norm = " << coef_sum << endl;
        gzlog << "b = \n" << b << endl;
        gzlog << "A = \n" << A << endl;
        // gzlog << "Reading pixel = (" << tex_coord.first << ", "
        //                              << tex_coord.second << ")" << endl;
        gzlog << "c = \n" << concentrations << endl;

        // gzlog << "Concentrations = (" << concentrations[0] / coef_sum << ", "
        //                               << concentrations[1] / coef_sum << ", "
        //                               << concentrations[2] / coef_sum << ")" << endl;
        // gzlog << "Texture Color = (" << tcolor.r << ", "
        //                              << tcolor.g << ", "
        //                              << tcolor.b << ")" << endl;
        // gzlog << "Material Color = (" << last_insert->second.second.r << ", "
        //                               << last_insert->second.second.g << ", "
        //                               << last_insert->second.second.b << ")" << endl;
      } else {
        blend = last_insert->second.first;
      }

      grid_points.emplace_back(
        static_cast<uint8_t>(last_insert->second.second.r), // truncates double to int (<256)
        static_cast<uint8_t>(last_insert->second.second.g),
        static_cast<uint8_t>(last_insert->second.second.b)
      );
      // WORKAROUND for OW-1194, TF has an incorrect transform for
      //            base_link (specific for atacama_y1a)
      center -= GridPositionType(-1.0, 0.0, 0.37);
      grid_points.back().x = static_cast<float>(center.X());
      grid_points.back().y = static_cast<float>(center.Y());
      grid_points.back().z = static_cast<float>(center.Z());
    }
  );

  // publish and latch grid data as a point cloud for visualization
  // only need to do this once because the grid is never modified
  publishPointCloud(&m_grid_pub, grid_points);

  gzlog << PLUGIN_NAME << ": Grid visualization now ready for viewing in Rviz."
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
  // compute an element-wise weighted average of all colors with the
  // concentrations acting as the weights
  // auto l1norm = std::accumulate(
  //   blend.getBlendMap().cbegin(), blend.getBlendMap().cend(),
  //   Color{0.0, 0.0, 0.0},
  //   [this]
  //   (Color value, MaterialBlend::BlendType::value_type const &p) {
  //     return Color {
  //       value.r + p.second;
  //     }
  //   }
  // );
  return std::accumulate(
    blend.getBlendMap().begin(), blend.getBlendMap().end(),
    Color{0.0, 0.0, 0.0},
    [this]
    (Color value, MaterialBlend::BlendType::value_type const &p) {
      const auto m = m_material_db->getMaterial(p.first);
      return Color {
        value.r + p.second * m.color.r,
        value.g + p.second * m.color.g,
        value.b + p.second * m.color.b
      };
    }
  );

  // return std::accumulate(
  //   blend.getBlendMap().begin(), blend.getBlendMap().end(),
  //   Color{0.0, 0.0, 0.0},
  //   [this]
  //   (Color value, MaterialBlend::BlendType::value_type const &p) {
  //     const auto m = m_material_db->getMaterial(p.first);
  //     return Color {
  //       value.r + p.second * (m.color.r - value.r),
  //       value.g + p.second * (m.color.g - value.g),
  //       value.b + p.second * (m.color.b - value.b)
  //     };
  //   }
  // );

  // additive color mixing version
  // treat each material's concentration as an alpha value
  // float f0 = 0.0f; // the previous concentration
  // return std::accumulate(
  //   blend.getBlendMap().begin(), blend.getBlendMap().end(),
  //   Color{0.0, 0.0, 0.0},
  //   [this, &f0]
  //   (Color value, MaterialBlend::BlendType::value_type const &p) {
  //     const auto m = m_material_db->getMaterial(p.first);
  //     float f1 = p.second;
  //     float fsum = 1 - (1 - f0) * (1 - f1);
  //     auto c = Color {
  //       (value.r * (1 - f1) + f1 * m.color.r) / fsum,
  //       (value.g * (1 - f1) + f1 * m.color.g) / fsum,
  //       (value.b * (1 - f1) + f1 * m.color.b) / fsum
  //     };
  //     f0 = f1;
  //     return c;
  //   }
  // );

}
