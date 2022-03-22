// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <algorithm>
#include <vector>

#include <ow_regolith/Contacts.h>

#include <ContactSensorPlugin.h>

using namespace ros;
using namespace gazebo;
using namespace ow_regolith;

using std::endl;
using std::string;
using std::set;
using std::vector;

const static string NODE_PREFIX = "contact_sensor_";

const static string PLUGIN_NAME = "ContactSensorPlugin";

const static string CONTACTS_TOPIC_PARAMETER = "topic";

GZ_REGISTER_SENSOR_PLUGIN(ContactSensorPlugin)

void ContactSensorPlugin::Load(sensors::SensorPtr sensor,
                                 sdf::ElementPtr sdf)
{
  // connnect parent sensor update signal to update handler
  m_parent_sensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
  if (!m_parent_sensor) {
    gzerr << PLUGIN_NAME << ": requires a sensor of type conatact as a parent" << endl;
    return;
  }
  m_update_connection = m_parent_sensor->ConnectUpdated(
    std::bind(&ContactSensorPlugin::onUpdate, this)
  );
  m_parent_sensor->SetActive(true);

  gzlog << PLUGIN_NAME << ": successfully loaded!" << endl;

  // setup ROS interface
  m_node_handle = std::make_unique<NodeHandle>(NODE_PREFIX + sdf->GetName());
  if (!sdf->HasElement(CONTACTS_TOPIC_PARAMETER))
    gzthrow(PLUGIN_NAME << ": Contacts report topic must be defiend");
  auto topic = sdf->Get<string>(CONTACTS_TOPIC_PARAMETER);
  m_pub_contacts = m_node_handle->advertise<Contacts>(topic, 1, true);
  gzlog << PLUGIN_NAME << ": publishing contacts on ROS topic " << topic << endl;
}

void ContactSensorPlugin::onUpdate()
{
  if (m_parent_sensor->GetCollisionCount() < 0)
    return;
  auto target_name = m_parent_sensor->GetCollisionName(0);
  set<string> current_contacts;
  for (auto const &contact : m_parent_sensor->Contacts(target_name)) {
    // collision1 is always the collider, not the terrain
    if (!contact.second.collision1) // check for nullptr
      continue;
    const auto link = contact.second.collision1->GetLink();
    const auto model = contact.second.collision1->GetModel();
    current_contacts.emplace(model->GetName() + "::" + link->GetName());
  }

  // publish only if the list of links in contact with sensor has changed
  if (m_links_in_contact != current_contacts) {
    m_links_in_contact = current_contacts;
    // publish list of all contacts
    Contacts msg;
    msg.names = vector(m_links_in_contact.begin(), m_links_in_contact.end());
    m_pub_contacts.publish(msg);
  }

}
