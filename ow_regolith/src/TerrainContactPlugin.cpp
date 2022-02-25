// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>

#include <TerrainContactPlugin.h>

#include <ow_regolith/TerrainContact.h>

using std::endl;

using namespace ros;
using namespace gazebo;
using namespace ow_regolith;

const static std::string NODE_NAME = "terrain_contact_node";

const static std::string PLUGIN_NAME = "TerrainContactPlugin";

const static std::string TOPIC_TERRAIN_COLLISION = "/ow_regolith/terrain_contact";

GZ_REGISTER_SENSOR_PLUGIN(TerrainContactPlugin)

void TerrainContactPlugin::Load(sensors::SensorPtr _sensor,
                                 sdf::ElementPtr _sdf)
{
  // connnect parent sensor update signal to update handler
  m_parent_sensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
  if (!m_parent_sensor) {
    gzerr << PLUGIN_NAME << ": requires a ContactSensor" << endl;
    return;
  }
  m_update_connection = m_parent_sensor->ConnectUpdated(
    std::bind(&TerrainContactPlugin::onUpdate, this)
  );
  m_parent_sensor->SetActive(true);

  gzlog << PLUGIN_NAME << ": successfully loaded!" << endl;

  // setup ROS interface
  m_node_handle = std::make_unique<NodeHandle>(NODE_NAME);
  m_pub_terrain_collision = m_node_handle->advertise<TerrainContact>(
    TOPIC_TERRAIN_COLLISION, 1
  );

  gzlog << PLUGIN_NAME << ": connected to ROS network!" << endl;
}

void TerrainContactPlugin::onUpdate()
{
  if (m_parent_sensor->GetCollisionCount() == 0) {
    gzwarn << PLUGIN_NAME << ": has no contact to observe" << endl;
    return;
  }

  auto terrain_contact_name = m_parent_sensor->GetCollisionName(0);
  TerrainContact msg;
  for (auto const &contact : m_parent_sensor->Contacts(terrain_contact_name)) {
    // DEBUG MSG
    gzlog << "Collision between[" << contact.first
              << "] and [" << terrain_contact_name << "]" << endl;

    // collision1 is always the collider, not the terrain
    if (!contact.second.collision1) // check for nullptr
      continue;
    auto link = contact.second.collision1->GetLink();
    auto model = contact.second.collision1->GetModel();
    // compile list of link names
    msg.names.push_back(model->GetName() + "::" + link->GetName());
    // compile list of total energies of each link
    msg.energies.push_back(link->GetWorldEnergy());
  }

  // publish names of links in contact with terrain
  if (msg.names.size() > 0)
    m_pub_terrain_collision.publish(msg);
}
