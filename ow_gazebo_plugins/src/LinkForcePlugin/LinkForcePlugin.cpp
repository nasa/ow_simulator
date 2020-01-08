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
  if(!loadLookupTable(_sdf->Get<string>("lookupTable")))
    return;

  // Listen to the update event. This event is broadcast every sim iteration.
  // If result goes out of scope updates will stop, so it is assigned to a member variable.
  m_updateConnection = event::Events::ConnectBeforePhysicsUpdate(std::bind(&LinkForcePlugin::OnUpdate, this));
}

bool LinkForcePlugin::loadLookupTable(string filename)
{
  ifstream infile(filename.c_str(), fstream::in);
  if (!infile.is_open()) {
    gzerr << "cannot open file: " << filename << endl;
    return false;
  }

  //string row;
  //while(infile >> row)
  //  cout << row << endl;

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

  // TODO: Get force and torque from lookup table here.

  // TODO: Figure out if this force applied at the CoG and where exactly we
  // want to apply it.
  m_link->AddRelativeForce(ignition::math::Vector3d(-20,0,0));
}
