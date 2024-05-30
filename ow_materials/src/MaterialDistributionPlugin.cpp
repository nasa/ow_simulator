// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>
#include <stdexcept>
#include <functional>
#include <unordered_map>

#include "boost/functional/hash.hpp"

#include "gazebo/rendering/rendering.hh"
#include "cv_bridge/cv_bridge.h"

#include "ow_materials/BulkExcavation.h"

#include "material_utils.h"
#include "point_cloud_util.h"
#include "MaterialDistributionPlugin.h"

using std::string, std::make_unique, std::endl, std::uint8_t, std::size_t,
      std::runtime_error, std::make_pair, std::unordered_map;

using namespace ow_materials;
using namespace gazebo;

const string PLUGIN_NAME = "MaterialDistributionPlugin";

const string NAMESPACE_MATERIALS = "/ow_materials/materials";

const string NODE_NAME = "/ow_materials/material_distribution_plugin";
const string TOPIC_MOD_DIFF_VISUAL = "/ow_dynamic_terrain/modification_differential/visual";
const string ROSPARAM_SIM_MAT_DIST = "/ow_materials/sim_material_distribution";

const string PARAMETER_CORNER_A         = "corner_a";
const string PARAMETER_CORNER_B         = "corner_b";
const string PARAMETER_CELL_SIDE_LENGTH = "cell_side_length";

const string PARAMETER_MATERIALS                  = "materials";
const string PARAMETER_MATERIALS_FILE             = "file";
const string PARAMETER_MATERIALS_CHILD            = "reference_color";
const string PARAMETER_MATERIALS_CHILD_ATTRIBUTE  = "material";

static float colorSpaceDistance(const Color &c1, const Color &c2)
{
  Color d{c2.r - c1.r, c2.g - c1.g, c2.b - c1.b};
  return std::sqrt(d.r * d.r + d.g * d.g + d.b * d.b);
}

void MaterialDistributionPlugin::computeDiffVolume(
  const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg) const
{
  auto diff_handle = cv_bridge::CvImageConstPtr();
  try {
    diff_handle = cv_bridge::toCvShare(msg->diff, msg);
  } catch (cv_bridge::Exception& e) {
    gzlog << "cv_bridge exception occurred while reading modification "
             "differential message: " << e.what() << std::endl;
    return;
  }
  const auto rows = diff_handle->image.rows;
  const auto cols = diff_handle->image.cols;
  const auto pixel_height = msg->height / rows;
  const auto pixel_width = msg->width / cols;
  const float pixel_area = static_cast<float>(pixel_height * pixel_width);
  float volume_displaced = 0.0;
  for (size_t i = 0; i != cols; ++i) {
    for (size_t j = 0; j != rows; ++j) {
      // change in height at pixel (x, y)
      auto dz = diff_handle->image.at<float>(i, j);
      if (dz >= 0.0) { // ignore unmodified or raised terrain
        continue;
      }
      // dz is always negative here, so subtract to get a positive volume
      volume_displaced -= dz * pixel_area;
    }
  }
  // create a uniform material of the first defined
  static const Blend UNIFORM_BLEND({{Material::id_min, 1.0}});
  handleVisualBulk(UNIFORM_BLEND, volume_displaced);
}

void MaterialDistributionPlugin::Load(physics::ModelPtr model,
                                      sdf::ElementPtr sdf)
{
  gzlog << PLUGIN_NAME << ": loading..." << endl;

  m_node_handle = make_unique<ros::NodeHandle>(NODE_NAME);

  // populate materials database
  try {
    m_material_db.populate_from_rosparams(NAMESPACE_MATERIALS);
  } catch (const MaterialConfigError &e) {
    gzerr << e.what() << endl;
    return;
  }
  gzlog << PLUGIN_NAME << ": Materials database populated with "
        << m_material_db.size() << " materials.\n";

  m_pub_bulk_excavation_visual = m_node_handle->advertise<BulkExcavation>(
    "/ow_materials/bulk_excavation/visual", 10, false);
  //// STUBBED FEATURE: reactivate to implement grinder terramechanics (OW-998)
  // m_pub_bulk_excavation_collision = m_node_handle->advertise<BulkExcavation>(
  //   "/ow_materials/bulk_excavation/collision", 10, false);

  // disable material distribution grid, and only compute volume displaced
  ros::param::param(ROSPARAM_SIM_MAT_DIST, m_grid_in_use, true);
  if (!m_grid_in_use) {
    gzlog << PLUGIN_NAME << ": Material distribution will not be simulated. "
          "To enable, launch a supported world with sim_material_distribution "
          "set to true." << endl;
    m_sub_modification_diff = m_node_handle->subscribe(TOPIC_MOD_DIFF_VISUAL,
      1, &MaterialDistributionPlugin::computeDiffVolume, this);
    gzlog << PLUGIN_NAME << ": Plugin successfully loaded." << endl;
    return;
  }

  // publish latched, because points will never change
  // message is published later on after the pointcloud has been generated
  m_pub_grid = m_node_handle->advertise<sensor_msgs::PointCloud2>(
    "/ow_materials/grid_points2", 1, true);


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
    m_grid = make_unique<AxisAlignedGrid<Blend>>(corner_a, corner_b,
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

  // get reference colors for mapping terrain texture to material compositions
  if (!sdf->HasElement(PARAMETER_MATERIALS)) {
    gzerr << PARAMETER_MATERIALS << " is required." << endl;
    return;
  }
  auto sdf_materials = sdf->GetElement(PARAMETER_MATERIALS);
  auto child = sdf_materials->GetFirstElement();
  while (child) {
    if (!child->HasAttribute(PARAMETER_MATERIALS_CHILD_ATTRIBUTE)) {
      gzerr << PARAMETER_MATERIALS_CHILD_ATTRIBUTE << " attribute is required "
            << "for each reference_color specified" << endl;
      return;
    }
    MaterialID id;
    try {
      id = m_material_db.getMaterialIdFromName(
        child->GetAttribute(PARAMETER_MATERIALS_CHILD_ATTRIBUTE)->GetAsString()
      );
    } catch (MaterialRangeError const &e) {
      gzerr << e.what()
            << " Verify that the material names in the reference_color entries "
               "defined in your world's model.sdf file match the names defined "
               "in your material's YAML file. See the Material Distribution "
               "section of the Environment Simulation page of the Wiki for "
               "a more in depth explanation." << endl;
      return;
    }
    // interpret value as a vector since it's the same shape as a color
    ignition::math::Color c;
    child->GetValue()->Get<ignition::math::Color>(c);
    m_reference_colors.emplace_back(id,
      Color{
        static_cast<float>(c.R() * 255.0),
        static_cast<float>(c.G() * 255.0),
        static_cast<float>(c.B() * 255.0)
      }
    );

    // acquire the next child
    child = child->GetNextElement(PARAMETER_MATERIALS_CHILD);
  }

  m_visual_integrator = make_unique<MaterialIntegrator>(
    m_node_handle.get(), m_grid.get(), TOPIC_MOD_DIFF_VISUAL,
    "/ow_materials/dug_points2",
    std::bind(&MaterialDistributionPlugin::handleVisualBulk, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&MaterialDistributionPlugin::interpolateColor, this,
      std::placeholders::_1)
  );

  //// STUBBED FEATURE: reactivate for grinder terramechanics (OW-998)
  // m_collision_integrator = make_unique<MaterialIntegrator>(
  //   m_node_handle.get(), m_grid.get(),
  //   "/ow_dynamic_terrain/modification_differential/collision",
  //   "/ow_materials/dug_points2",
  //   std::bind(&MaterialDistributionPlugin::handleCollisionBulk, this,
  //     std::placeholders::_1, std::placeholders::_2),
  //   std::bind(&MaterialDistributionPlugin::interpolateColor, this,
  //     std::placeholders::_1)
  // );

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
    // disable callback by deleting its handle
    delete m_temp_render_connection.release();
    return;
  }
  Ogre::Image albedo;
  texture->convertToImage(albedo);
  gzlog << PLUGIN_NAME << ": Heightmap albedo texture acquired.\n";
  gzlog << PLUGIN_NAME << ": Plugin successfully loaded." << endl;
  // the process can take as long as 10 seconds, so we spin it off in its own
  // thread to not hold up loading the rest of Gazebo
  std::thread t(
    &MaterialDistributionPlugin::populateGrid, this,
    std::move(albedo), terrain
  );
  t.detach();
  // the callback's sole purpose has been fulfilled, so we can disable it by
  // deleting its event handle
  delete m_temp_render_connection.release();
}

void MaterialDistributionPlugin::populateGrid(Ogre::Image albedo,
                                              Ogre::Terrain *terrain)
{
  gzlog << PLUGIN_NAME << ": Populating grid from heightmap albedo..." << endl;
  // time the following operation
  auto start_time = ros::WallTime::now();

  // defines image coordinates that can be an unordered_map key
  struct TextureCoords {
    size_t x, y;
    inline bool operator==(const TextureCoords &other) const {
      return x == other.x && y == other.y;
    };
    struct hasher {
      inline size_t operator()(const TextureCoords &p) const {
        size_t seed = 0;
        boost::hash_combine(seed, p.x);
        boost::hash_combine(seed, p.y);
        return seed;
      };
    };
  };
  // simple struct for more readable code
  struct BlendAndColor { Blend blend; Color color; };
  using AlbedoMaterialMap = unordered_map<TextureCoords, BlendAndColor,
                                          TextureCoords::hasher>;
  // this map enables z-column-wise modification of the grid without having to
  // compute the material blend for an image coordinate more than once
  AlbedoMaterialMap albedo_blends;
  AlbedoMaterialMap::const_iterator last_insert = albedo_blends.cbegin();
  // point cloud object that will be published after the loop
  pcl::PointCloud<pcl::PointXYZRGB> grid_points;
  m_grid->runForEach(
    [&]
    (Blend &blend, GridPositionType center) {
      // acquire x and y in terrain space (between 0 and 1)
      auto tpos = terrain->convertPosition(
        Ogre::Terrain::Space::WORLD_SPACE,
        Ogre::Vector3(center.X(), center.Y(), center.Z()),
        Ogre::Terrain::Space::TERRAIN_SPACE
      );

      // skip grid as it is not within the terrain
      if (terrain->getHeightAtTerrainPosition(tpos.x, tpos.y) <= center.Z()) {
        return;
      }

      TextureCoords tex_coord{
        static_cast<size_t>(tpos.x * albedo.getWidth()),
        // y terrain coordinate must be flipped to work with image coordinates
        static_cast<size_t>((1.0f - tpos.y) * albedo.getHeight())
      };

      auto it = albedo_blends.find(tex_coord);
      if (it != albedo_blends.end()) {
        // blend has already been done for this coordinate, reuse the result
        blend = last_insert->second.blend;
      } else {
        auto ogre_color = albedo.getColourAt(tex_coord.x, tex_coord.y, 0u);
        Color tex_color(ogre_color.r * 255.0f,
                        ogre_color.g * 255.0f,
                        ogre_color.b * 255.0f);
        std::vector<float> inverse_dist;
        for (auto rc : m_reference_colors) {
          if (rc.second == tex_color) {
            // perfect color match, size of vector is used to infer this later
            blend.add(rc.first, 1.0f);
            break;
          }
          inverse_dist.push_back(1 / colorSpaceDistance(tex_color, rc.second));
        }
        if (inverse_dist.size() == m_reference_colors.size()) {
          // a perfect color match did not occur
          float sum = std::reduce(inverse_dist.begin(), inverse_dist.end());
          for (size_t i = 0; i != inverse_dist.size(); ++i) {
            blend.add(m_reference_colors[i].first, inverse_dist[i] / sum);
          }
        }
        last_insert = albedo_blends.emplace_hint(last_insert,
          make_pair(tex_coord, BlendAndColor{blend, interpolateColor(blend)})
        );
      }

      grid_points.emplace_back(
        // truncates double to int (<256)
        static_cast<uint8_t>(last_insert->second.color.r),
        static_cast<uint8_t>(last_insert->second.color.g),
        static_cast<uint8_t>(last_insert->second.color.b)
      );
      grid_points.back().x = static_cast<float>(center.X());
      grid_points.back().y = static_cast<float>(center.Y());
      grid_points.back().z = static_cast<float>(center.Z());
    }
  );

  gzlog << PLUGIN_NAME << ": Grid generation took "
        << (ros::WallTime::now() - start_time).toSec() << " seconds." << endl;

  // publish and latch grid data as a point cloud for visualization
  // only need to do this once because the grid is never modified
  publishPointCloud(&m_pub_grid, grid_points);

  gzlog << PLUGIN_NAME << ": Grid visualization now ready for viewing in Rviz."
        << endl;
}

void MaterialDistributionPlugin::handleVisualBulk(Blend const &blend,
                                                  double volume) const
{
  Bulk excavated_bulk(blend, volume);
  BulkExcavation msg;
  try {
    msg = bulkToBulkExcavationMsg(excavated_bulk, m_material_db);
  } catch (MaterialRangeError const &e) {
    gzerr << e.what() << endl;
    return;
  }
  msg.header.stamp = ros::Time::now();
  m_pub_bulk_excavation_visual.publish(msg);
}

//// STUBBED FEATURE: reactivate for grinder terramechanics (OW-998)
// void MaterialDistributionPlugin::handleCollisionBulk(Blend const &blend,
//                                                      double volume) const
// {

// }

Color MaterialDistributionPlugin::interpolateColor(Blend const &blend) const
{
  return std::accumulate(
    blend.getComposition().begin(), blend.getComposition().end(),
    Color{0.0, 0.0, 0.0},
    [this]
    (Color value, CompositionType::value_type const &p) {
      const auto m = m_material_db.getMaterial(p.first);
      return Color {
        value.r + p.second * (m.visualize_color.r - value.r),
        value.g + p.second * (m.visualize_color.g - value.g),
        value.b + p.second * (m.visualize_color.b - value.b)
      };
    }
  );
}
