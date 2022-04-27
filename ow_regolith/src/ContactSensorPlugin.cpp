// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <algorithm>
#include <vector>

#include <ow_regolith/Contacts.h>

#include <ContactSensorPlugin.h>

using namespace ow_regolith;

using namespace ros;
using namespace gazebo;

using std::string, std::set, std::vector, std::bind, std::dynamic_pointer_cast,
      std::endl, std::regex_match, std::begin, std::end, std::regex,
      std::make_unique;

const static string NODE_PREFIX = "contact_sensor_";

const static string PLUGIN_NAME = "ContactSensorPlugin";

const static string PARAMETER_TOPIC = "topic";
const static string PARAMETER_REPORT_ONLY = "report_only";

GZ_REGISTER_SENSOR_PLUGIN(ContactSensorPlugin)

void ContactSensorPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  // connnect parent sensor update signal to update handler
  m_parent_sensor =
    dynamic_pointer_cast<sensors::ContactSensor>(sensor);
  if (!m_parent_sensor)
    gzthrow(PLUGIN_NAME << ": requires a sensor of type conatact as a parent");
  m_update_connection = m_parent_sensor->ConnectUpdated(
    bind(&ContactSensorPlugin::onUpdate, this)
  );
  m_parent_sensor->SetActive(true);

  // get plugin parameters
  if (!sdf->HasElement(PARAMETER_TOPIC)) // required
    gzthrow(PLUGIN_NAME << ": Contacts report topic must be defined");
  auto topic = sdf->Get<string>(PARAMETER_TOPIC);
  if (sdf->HasElement(PARAMETER_REPORT_ONLY)) { // optional
    m_report_only_set = true;
    const auto regex_str = sdf->Get<string>(PARAMETER_REPORT_ONLY);
    m_report_only = regex(regex_str);
    gzlog << PLUGIN_NAME
          << ": reporting only link names that match the regex pattern \""
          << regex_str << "\"\n";
  }

  // setup ROS interface
  m_node_handle = make_unique<NodeHandle>(NODE_PREFIX + sdf->GetName());
  m_pub_contacts = m_node_handle->advertise<Contacts>(topic, 1, true);
  gzlog << PLUGIN_NAME << ": Contacts will be reported on ROS topic "
                       << topic << '\n';

  gzlog << PLUGIN_NAME << ": successfully loaded!" << endl;
}

void ContactSensorPlugin::onUpdate()
{
  if (m_parent_sensor->GetCollisionCount() <= 0)
    return;
  const auto target_name = m_parent_sensor->GetCollisionName(0);
  set<string> current_contacts;
  for (auto const &contact : m_parent_sensor->Contacts(target_name)) {
    // collision1 is always the collider, not the terrain
    if (!contact.second.collision1) // check for nullptr
      continue;
    const auto link = contact.second.collision1->GetLink();
    const auto model = contact.second.collision1->GetModel();
    const auto link_name = model->GetName() + "::" + link->GetName();
    if (m_report_only_set && !regex_match(link_name, m_report_only))
      continue;
    current_contacts.emplace(link_name);
  }

  // publish only if the list of links in contact with sensor has changed
  if (m_links_in_contact != current_contacts) {
    m_links_in_contact = current_contacts;
    // publish list of all contacts
    Contacts msg;
    msg.link_names = vector(begin(m_links_in_contact), end(m_links_in_contact));
    m_pub_contacts.publish(msg);
  }
}
