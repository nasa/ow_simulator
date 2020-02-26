/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
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
    gzerr << "LinkForcePlugin: you must specify a <link> element." << endl;
    return;
  }
  string linkName = _sdf->Get<string>("link");
  m_link = _model->GetLink(linkName);
  if(!m_link) {
    gzerr << "LinkForcePlugin: specified link is invalid." << endl;
    return;
  }

  if (!_sdf->HasElement("lookupTable")) {
    gzerr << "LinkForcePlugin: you must specify a filename in a <lookupTable> element." << endl;
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
    gzerr << "LinkForcePlugin: cannot open file: " << filename << endl;
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
    //m_forceRows.push_back(ForceRow(row));

    ForceRow f(row);
    auto& vec = m_forcesMap[f.m_m][f.m_d][f.m_p][f.m_rho];
    vec.insert(vec.end(), f.m_force_torque.begin(), f.m_force_torque.end());
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
    gzerr << "LinkForcePlugin::OnUpdate - scene pointer is NULL" << endl;
    return;
  }
  rendering::Heightmap* heightmap = scene->GetHeightmap();
  if (!heightmap) {
    gzerr << "LinkForcePlugin::OnUpdate - heightmap pointer is NULL" << endl;
    return;
  }

  // Center of Gravity (CoG) pose gives us the position defined in the
  // <link><inertial><origin> part of the URDF.
  ignition::math::Pose3d cogpose = m_link->WorldCoGPose();

  // Get altitude of center of gravity of link. This is not exact because
  // the link touches the surface before its center of gravity touches.
  // TODO: Is this good enough?
  double altitude = cogpose.Pos().Z() - heightmap->Height(cogpose.Pos().X(), cogpose.Pos().Y());
  if(altitude >= 0){
    return;
  }

  // Get force and torque from lookup table here.
  std::vector<float> forces;
  GetForces(1, 6.0, 1, 0.2f, forces);

  // TODO: Figure out if this force applied at the CoG and where exactly we
  // want to apply it.
  m_link->AddRelativeForce(ignition::math::Vector3d(forces[0], forces[1], forces[2]));
  m_link->AddRelativeTorque(ignition::math::Vector3d(forces[3], forces[4], forces[5]));
}

bool LinkForcePlugin::GetForces(int material, float depth, int pass, float rho,
                                std::vector<float>& out_forces)
{
  // find this material
  auto depth_map = m_forcesMap[material];

  // Find the nearest map associated with this floating point depth
  auto pass_map = depth_map.rbegin()->second;
  {
    auto not_less_than_iter = depth_map.lower_bound(depth);
    if(not_less_than_iter != depth_map.end()) {
      if(not_less_than_iter == depth_map.begin())
        pass_map = not_less_than_iter->second;
      else {
        auto previous_iter = std::prev(not_less_than_iter);
        if ((depth - previous_iter->first) < (not_less_than_iter->first - depth))
          pass_map = previous_iter->second;
        else
          pass_map = not_less_than_iter->second;
      }
    }
  }

  // find this pass
  auto rho_map = pass_map[pass];

  // Find the nearest forces vector associated with this floating point rho
  auto forces = rho_map.rbegin()->second;
  {
    auto not_less_than_iter = rho_map.lower_bound(depth);
    if(not_less_than_iter != rho_map.end()) {
      if(not_less_than_iter == rho_map.begin())
        forces = not_less_than_iter->second;
      else {
        auto previous_iter = std::prev(not_less_than_iter);
        if ((depth - previous_iter->first) < (not_less_than_iter->first - depth))
          forces = previous_iter->second;
        else
          forces = not_less_than_iter->second;
      }
    }
  }

  if(forces.size() != 6) {
    gzerr << "LinkForcePlugin::GetForces - forces vector has size = "
          << forces.size() << ". It should be 6." << endl;
    return false;
  }

  out_forces = forces;

  return true;
}
