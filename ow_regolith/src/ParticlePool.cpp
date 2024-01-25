// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <stdexcept>
#include <sstream>

#include "sdf_utility.h"
#include "ServiceClientFacade.h"

#include "geometry_msgs/Vector3.h"

#include "gazebo_msgs/BodyRequest.h"

#include "ParticlePool.h"

using namespace ow_regolith;

using namespace sdf_utility;

using std::string, std::vector, std::stringstream, std::endl;

using ignition::math::Vector3d, ignition::math::Quaterniond,
      ignition::math::Pose3d;

// Time before a BulkExcavation is sent on /ow_materials/material_ingested
// following receipt of a removal request where ingested=True
// NOTE: This interval intentionally matches the interval in
//  DockIngestSampleServer in ow_lander/src/ow_lander/actions.py such that
//  there will only be one BulkExcavation message on this topic per call to
//  DockIngestSample.
const ros::Duration CONSOLIDATE_INGESTED_TIMEOUT = ros::Duration(3.0);

const string NOT_INITIALIZED_ERR{
  "ParticlePool has not yet been initialized!"
};

static geometry_msgs::Vector3 toVector3Message(const Vector3d &vec)
{
  geometry_msgs::Vector3 result;
  result.x = vec.X();
  result.y = vec.Y();
  result.z = vec.Z();
  return result;
}

ParticlePool::ParticlePool()
{
  // do nothing
}

bool ParticlePool::initialize(const string name,
                              gazebo::physics::WorldPtr world,
                              const string &model_uri, ros::NodeHandle *nh)
{
  // prepare world to contain a particle pool
  if (!world) {
    gzerr << "Invalid world pointer" << endl;
    return false;
  }
  m_world = world;
  m_name = name;
  // Insert a model to contain regolith links into world
  // NOTE: The content of this string just needs to be a valid and empty
  //  SDF model.
  stringstream ss;
  ss << "<?xml version=\"1.0\" ?>"
        "<sdf version=\"1.5\">"
        "  <model name=\"" << m_name << "\">"
        "    <self_collide>true</self_collide>"
        "  </model>"
        "</sdf>";
  m_world->InsertModelString(ss.str());
  // acquire SDF data from URI path
  string sdf_string;
  if (!getSdfFromUri(model_uri, sdf_string)) {
    gzerr << "Failed to load particle SDF file" << endl;
    return false;
  }
  parseSdf(sdf_string, m_model_sdf);
  if (!getModelLink(m_model_sdf, m_link_sdf)) {
    gzerr << "There is no link in particle SDF file" << endl;
    return false;
  }
  // setup ROS and gazebo facilities like events, timers, and publishers
  // NOTE: The actual insertion is deferred for some amount of Gazebo cycles.
  //  This means to get a pointer to the inserted model, we must check for its
  //  existence in the world until it is found. This is the purpose of
  //  m_temp_event.
  m_temp_event = std::make_unique<gazebo::event::ConnectionPtr>(
    gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&ParticlePool::getModelFromWorld, this)
    )
  );
  m_update_event = gazebo::event::Events::ConnectBeforePhysicsUpdate(
    std::bind(&ParticlePool::onUpdate, this)
  );
  // advertise a topic for communicating ingested sample contents
  m_pub_material_ingested = nh->advertise<ow_materials::BulkExcavation>(
    "/ground_truth/material_ingested", 1, true);
  m_consolidate_ingested_timeout = nh->createTimer(CONSOLIDATE_INGESTED_TIMEOUT,
    &ParticlePool::onConsolidateIngestedTimeout, this, true, false);

  return true;
}

string ParticlePool::spawn(const Vector3d &position,
                           const string &reference_frame,
                           const ow_materials::Bulk &bulk)
{
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return "";
  }
  // ensure each created particle has a unique name
  static unsigned int spawn_count = 0u;
  stringstream ss;
  ss << "p" << spawn_count++;
  const auto link_name = ss.str();
  // create particle as a child of the ParticlePool's model
  auto regolith_link = m_model->CreateLink(link_name);
  if (!regolith_link) {
    gzerr << "CreateLink call failed" << endl;
    return "";
  }
  regolith_link->Load(m_link_sdf);
  regolith_link->SetName(link_name);
  regolith_link->Init();
  if (reference_frame == "" || reference_frame == "world") {
    regolith_link->SetWorldPose(Pose3d(position, Quaterniond::Identity));
  } else {
    // find world pose relative to requested reference frame
    const auto reference = m_world->EntityByName(reference_frame);
    if (!reference) {
      gzerr << "Failed to find the entity that corresponds to the requested "
               "reference frame, " << reference_frame << "." << endl;
      return "";
    }
    regolith_link->SetWorldPose(
      reference->WorldPose() * Pose3d(position, Quaterniond::Identity)
    );
  }
  m_active_models.emplace(link_name, Particle{regolith_link, bulk});
  return link_name;
}

void ParticlePool::remove(const vector<string> &link_names, bool ingested)
{
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return;
  }

  // TROUBLESHOOTING: The following is mess because of much troubleshooting to
  //  find a work around to gazebosim/gazebo-classic Issue 2627.
  //  (https://github.com/gazebosim/gazebo-classic/issues/2627)

  gzlog << "m_model = " << m_model << "\n";

  // DEBUG
  {
    stringstream ss;
    ss << "m_model children = ";
    for (int i = 0; i < m_model->GetChildCount(); ++i) {
      ss << m_model->GetChild(i)->GetName();
      ss << ", ";
    }
    gzlog << ss.str() << endl;
  }

  for (const auto ln : link_names) {
    // m_active_models.at(ln).link->Fini();
    // gzlog << "FINE" << endl;
    // m_active_models.at(ln).link->GetSDF()->GetElement("self_collide")
    //   ->GetValue()->SetUpdateFunc([]() -> bool{return true;});
    // m_active_models.at(ln).link->Reset();
    gzlog << "FINE" << endl;
    m_model->RemoveChild(ln);
    gzlog << "FINE" << endl;
    m_active_models.at(ln).link->Load(m_link_sdf);
    // gzlog << "FINE" << "\n";
    // m_active_models.at(ln).link->Fini();
    // m_model->Fini();
    // m_active_models.erase(ln);
  }
  gzlog << "FINE" << endl;

  // DEBUG
  {
    stringstream ss;
    ss << "m_model children = ";
    for (int i = 0; i < m_model->GetChildCount(); ++i) {
      ss << m_model->GetChild(i)->GetName();
      ss << ", ";
    }
    gzlog << ss.str() << endl;
  }

  // if (link_names.empty()) {
  //   return {};
  // }
  // vector<string> not_removed;
  // DeleteModel msg;
  // // delete one or multiple models as provided in link_names
  // for (auto const &n : link_names) {
  //   ow_materials::Bulk bulk;
  //   try {
  //     msg.request.model_name = m_active_models.at(n).model_name;
  //     bulk = m_active_models.at(n).bulk;
  //   } catch(out_of_range &_err) {
  //     not_removed.push_back(n);
  //     continue;
  //   }
  //   if (removeModel(msg)) {
  //     m_active_models.erase(n);
  //     if (ingested) {
  //       m_ingested_bulk.mix(bulk);
  //       resetConsolidatedIngestedTimeout();
  //     }
  //   } else {
  //     not_removed.push_back(n);
  //   }
  // }
  // return not_removed;
}

void ParticlePool::clear() {
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return;
  }
  m_model->RemoveChildren();
  m_active_models.clear();
  clearAllForces();
  // vector<string> not_removed;
  // DeleteModel msg;
  // // delete all regolith models
  // auto it = begin(m_active_models);
  // while (it != end(m_active_models)) {
  //   msg.request.model_name = it->second.model_name;
  //   if (removeModel(msg)) {
  //     it = m_active_models.erase(it);
  //   } else {
  //     // do not remove from active list if removal failed
  //     not_removed.push_back((it++)->first);
  //   }
  // }
  // return not_removed;
}

bool ParticlePool::applyMaintainedForce(const string &link_name,
                                        const Vector3d &force) {
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return false;
  }
  try {
    m_maintained_forces.push_back({m_active_models.at(link_name).link, force});
  } catch (std::out_of_range err) {
    gzerr << "Attempted to apply force to link that does not exist." << endl;
    return false;
  }
  return true;
}

void ParticlePool::clearAllForces()
{
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return;
  }
  m_maintained_forces.clear();
}

// bool ParticlePool::removeModel(DeleteModel &msg)
// {
  // if (!m_gz_delete_model.call(msg)) {
  //   ROS_WARN("Failed to remove model %s, response: %s",
  //     msg.request.model_name.c_str(), msg.response.status_message.c_str()
  //   );
  //   return false;
  // }
  // return true;
// }

void ParticlePool::getModelFromWorld()
{
  m_model = m_world->ModelByName(m_name);
  if (m_model != nullptr) {
    // Model was found; disable further callbacks.
    delete m_temp_event.release();
  }
}

void ParticlePool::onUpdate() const
{
  for (const auto &x : m_maintained_forces) {
    x.link->AddLinkForce(x.force);
  }
}

void ParticlePool::onConsolidateIngestedTimeout(const ros::TimerEvent&)
{
  m_pub_material_ingested.publish(
    m_ingested_bulk.generateExcavationBulkMessage());
  m_ingested_bulk.clear();
}

void ParticlePool::resetConsolidatedIngestedTimeout()
{
  m_consolidate_ingested_timeout.stop();
  m_consolidate_ingested_timeout.setPeriod(CONSOLIDATE_INGESTED_TIMEOUT);
  m_consolidate_ingested_timeout.start();
}
