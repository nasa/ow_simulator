// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "LinkForcePlugin.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(LinkForcePlugin)

LinkForcePlugin::LinkForcePlugin() :
  ModelPlugin()
{
}

LinkForcePlugin::~LinkForcePlugin()
{
}

void LinkForcePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("link")) {
    gzerr << "Load - you must specify a <link> element." << endl;
    return;
  }
  string linkName = _sdf->Get<string>("link");
  m_link = _model->GetLink(linkName);
  if(!m_link) {
    gzerr << "Load - specified link is invalid." << endl;
    return;
  }

  if (!_sdf->HasElement("lookupTable")) {
    gzerr << "Load - you must specify a filename in a <lookupTable> element." << endl;
    return;
  }
  if(!LoadLookupTable(_sdf->Get<string>("lookupTable")))
    return;

  // Listen to the update event. This event is broadcast every sim iteration.
  // If result goes out of scope updates will stop, so it is assigned to a member variable.
  m_updateConnection = event::Events::ConnectBeforePhysicsUpdate(std::bind(&LinkForcePlugin::OnUpdate, this));
}

bool LinkForcePlugin::LoadLookupTable(string filename)
{
  ifstream infile(filename.c_str(), fstream::in);
  if (!infile.is_open()) {
    gzerr << "LoadLookupTable - cannot open file: " << filename << endl;
    return false;
  }

  // Read in force and torque lookup table
  CSVRow row;
  bool first = true;
  while(infile >> row) {
    if(first) {
      first = false;
      continue;
    }

    // Store lookup table row in map
    ForceRow f(row);
    if(f.m_force_torque.size() != 6) {
      gzerr << "LoadLookupTable - force/torque vector size = "
            << f.m_force_torque.size() << ". Should be 6." << endl;
      continue;
    }
    auto& vec = m_forcesMap[f.m_m][f.m_d][f.m_p][f.m_rho];
    if(vec.size() != 0) {
      gzerr << "LoadLookupTable - duplicate key = "
            << f.m_m << ","<< f.m_d << ","<< f.m_p << ","<< f.m_rho << endl;
      continue;
    }
    vec = f.m_force_torque;
  }
  infile.close();

  return true;
}

void LinkForcePlugin::OnUpdate()
{
  // Get heightmap pointer so we can look up its height values.
  // If we stop using a heightmap for terrain, we will need a new solution for this.
  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    gzerr << "OnUpdate - scene pointer is NULL" << endl;
    return;
  }
  rendering::Heightmap* heightmap = scene->GetHeightmap();
  if (!heightmap) {
    gzerr << "OnUpdate - heightmap pointer is NULL" << endl;
    return;
  }

  // Center of Gravity (CoG) pose gives us the position defined in the
  // <link><inertial><origin> part of the URDF.
  ignition::math::Pose3d cogpose = m_link->WorldCoGPose();

  // Get altitude of center of gravity of link. This is not exact because
  // the link touches the surface before its center of gravity touches.
  // TODO: Is this good enough?
  double altitude = cogpose.Pos().Z() - heightmap->Height(cogpose.Pos().X(), cogpose.Pos().Y());
  if(altitude >= 0) {
    return;
  }

  // Get force and torque from lookup table here. Altitude (m) is converted to
  // depth (mm).
  // TODO: Get proper index for material, maybe depth, pass, and maybe rho.
  // We will likely get them from topics sent by autonomy.
  std::vector<float> forces;
  if(!GetForces(1, -altitude, 1, 0.0002f, forces)) {
    return;
  }

  // Apply force and torque at the link's CoG
  m_link->AddRelativeForce(ignition::math::Vector3d(forces[0], forces[1], forces[2]));
  m_link->AddRelativeTorque(ignition::math::Vector3d(forces[3], forces[4], forces[5]));
}

bool LinkForcePlugin::GetForces(int material, float depth, int pass, float rho,
                                std::vector<float>& out_forces)
{
  try {
    // Find this material
    auto depth_map = m_forcesMap.at(material);

    // Find the nearest map associated with this floating point depth
    auto pass_map = FindValueForClosestKey(depth_map, depth);

    // Find this pass
    auto rho_map = pass_map.at(pass);

    // Find the nearest forces vector associated with this floating point rho
    auto forces = FindValueForClosestKey(rho_map, rho);

    if(forces.size() != 6) {
      gzerr << "GetForces - forces vector has size = "
            << forces.size() << ". It should be 6." << endl;
      return false;
    }

    out_forces = forces;
  }
  catch (const std::out_of_range& oor) {
    gzerr << "GetForces - Out of range exception: " << oor.what() << endl;
    return false;
  }
  catch ( ... ) {
    gzerr << "GetForces -  Exception thrown." << endl;
    return false;
  }

  return true;
}

template <typename key_t, typename value_t>
value_t LinkForcePlugin::FindValueForClosestKey(const std::map<key_t, value_t>& inmap, const key_t& key)
{
  // Start with last value in map
  value_t value = inmap.rbegin()->second;
  // Get lowest match for key, not less than the key
  auto not_less_than_iter = inmap.lower_bound(key);
  if(not_less_than_iter != inmap.end()) {
    if(not_less_than_iter == inmap.begin())
      value = not_less_than_iter->second;
    else {  // Find value if key is in between keys in the map
      auto previous_iter = std::prev(not_less_than_iter);
      if ((key - previous_iter->first) < (not_less_than_iter->first - key))
        value = previous_iter->second;
      else
        value = not_less_than_iter->second;
    }
  }

  return value;
}
