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
#include <Eigen/LU>

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

  gzlog << "FINE" << endl;

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
          << e.what() << " could not be found." << endl;
    delete m_temp_render_connection.release();
    return;
  }

  gzlog << "FINE" << endl;

  Ogre::Image albedo;

  // the following causes a segmentation fault???
  texture->convertToImage(albedo);

  // auto color = albedo.getColourAt(0, 0, 0);
  // gzlog << "COLOR (0, 0, 0) = " << color.r << " " << color.g << " " << color.b << "\n";
  // gzlog << "albedo.getDepth()  = " << albedo.getDepth() << endl;
  // gzlog << "albedo.getHeight() = " << albedo.getHeight() << endl;
  // gzlog << "albedo.getWidth()  = " << albedo.getWidth() << endl;
  // albedo.save("/usr/local/home/tstucky/ow/beta_ws/test_albedo.png");

  gzlog << PLUGIN_NAME << ": Heightmap albedo texture acquired." << endl;

  // the process takes about 6 seconds, so we spin it off in its own thread to
  // not hold up loading the rest of Gazebo
  // std::thread t(&MaterialDistributionPlugin::populateGrid, this, std::ref(heightmap));
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

  pcl::PointCloud<pcl::PointXYZRGB> grid_points;

  // TODO: use unordered_map instead; a custom pair hash function is required
  using AlbedoMaterialMap = std::map<std::pair<size_t, size_t>,
                                     std::pair<MaterialBlend, Color>>;
  AlbedoMaterialMap albedo_blends;
  AlbedoMaterialMap::const_iterator last_insert = std::begin(albedo_blends);

  m_grid->runForEach(
    [&basis, &albedo, &terrain, &grid_points, &albedo_blends, &last_insert, this]
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
        // pack into 3xN matrix and N vector
        // Eigen::Matrix3Xf A(basis.size());
        // for (size_t i = 0; i != A.cols(); ++i) {
        //   A.col(i) = static_cast<Eigen::Vector3f>(basis[i].second);
        // }
        // std::vector<float> temp;
        // for (auto const &x : basis)
        //   temp.push_back(x.second.r);
        // for (auto const &x : basis)
        //   temp.push_back(x.second.g);
        // for (auto const &x : basis)
        //   temp.push_back(x.second.b);
        constexpr size_t COLOR_COUNT = 3;
        std::vector<float> temp(COLOR_COUNT * basis.size());
        for (size_t i = 0; i != basis.size(); ++i) {
          temp[i]                   = basis[i].second.r;
          temp[i + COLOR_COUNT]     = basis[i].second.g;
          temp[i + 2 * COLOR_COUNT] = basis[i].second.b;
        }
        Eigen::Matrix3Xf A = Eigen::Matrix3Xf::Map(&temp[0], COLOR_COUNT, basis.size());
        Eigen::Vector3f b = static_cast<Eigen::Vector3f>(tcolor);
        // solve
        Eigen::ColPivHouseholderQR<Eigen::Matrix3Xf> dec(A);
        Eigen::VectorXf concentrations = dec.solve(b);
        // normalize before adding into the blend to save on computation
        concentrations.normalize();
        // save resulting concentrations to the material blend for the cell
        for (size_t i = 0; i != A.cols(); ++i) {
          blend.add(basis[i].first, concentrations[i]);
        }
        last_insert = albedo_blends.emplace_hint(
          last_insert,
          make_pair(
            tex_coord,
            make_pair(blend, tcolor)
          )
        );
        // gzlog << PLUGIN_NAME << ": reading pixel ( "
        //       << tex_coord.first << ", "
        //       << tex_coord.second << " )" << endl;
      }

      // publish and latch grid data as a point cloud for visualization
      // const Color c = interpolateColor(b);
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

  // gzlog << PLUGIN_NAME
  //       << ": Heightmap albedo texture decomposed into materials." << endl;

  // gzlog << PLUGIN_NAME << ": Populating grid visualization data..." << endl;

  // pcl::PointCloud<pcl::PointXYZRGB> grid_points;
  // // publish and latch grid data as a point cloud for visualization
  // m_grid->runForEach(
  //   [&grid_points, this]
  //   (MaterialBlend const &b, GridPositionType center) {
  //     const Color c = interpolateColor(b);
  //     grid_points.emplace_back(
  //       static_cast<uint8_t>(c.r), // truncates double to int (<256)
  //       static_cast<uint8_t>(c.g),
  //       static_cast<uint8_t>(c.b)
  //     );

  //     // WORKAROUND for OW-1194, TF has an incorrect transform for
  //     //            base_link (specific for atacama_y1a)
  //     center -= GridPositionType(-1.0, 0.0, 0.37);

  //     grid_points.back().x = static_cast<float>(center.X());
  //     grid_points.back().y = static_cast<float>(center.Y());
  //     grid_points.back().z = static_cast<float>(center.Z());
  //   }
  // );

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
  return std::accumulate(blend.getBlendMap().begin(), blend.getBlendMap().end(),
    Color{0.0, 0.0, 0.0},
    [this]
    (Color value, MaterialBlend::BlendType::value_type const &p) {
      const auto m = m_material_db->getMaterial(p.first);
      // TODO: define + and * operators for color
      return Color {
        value.r + p.second * m.color.r,
        value.g + p.second * m.color.g,
        value.b + p.second * m.color.b
      };
    }
  );
}
