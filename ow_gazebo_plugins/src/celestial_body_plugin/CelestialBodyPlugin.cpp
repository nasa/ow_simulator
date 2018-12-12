/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#include "CelestialBodyPlugin.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(CelestialBodyPlugin)

CelestialBodyPlugin::CelestialBodyPlugin() :
  ModelPlugin(),
  m_transformListener(m_tfBuffer)
{
  m_timer.Start();
}

CelestialBodyPlugin::~CelestialBodyPlugin()
{
}


void CelestialBodyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  m_model = _model;

  if (!_sdf->HasElement("frame")) {
    gzerr << "CelestialBodyPlugin: you must specify a frame element." << endl;
    return;
  }
  m_frame = _sdf->Get<string>("frame");

  if (!_sdf->HasElement("radius")) {
    gzerr << "CelestialBodyPlugin: you must specify a radius." << endl;
    return;
  }
  m_radius = _sdf->Get<double>("radius");

  if (!_sdf->HasElement("render_distance")) {
    gzerr << "CelestialBodyPlugin: you must specify a render_distance." << endl;
    return;
  }
  m_renderDistance = _sdf->Get<double>("render_distance");

  // Listen to the update event. This event is broadcast every sim iteration.
  // If result goes out of scope. updates will stop, so it assigned to a member variable.
  m_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CelestialBodyPlugin::OnUpdate, this));
}

void CelestialBodyPlugin::OnUpdate()
{
  // Only continue if a second has elapsed
  if (m_timer.GetElapsed().Double() < 1.0)
  {
    return;
  }
  m_timer.Reset();
  m_timer.Start();

  // Get most recent transform
  geometry_msgs::TransformStamped sXform;
  try
  {
    sXform = m_tfBuffer.lookupTransform("site", m_frame, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("CelestialBodyPlugin::OnUpdate - %s", ex.what());
    return;
  }

  geometry_msgs::Vector3& v = sXform.transform.translation;
  math::Vector3 pos(v.x, v.y, v.z);
  const double distance = pos.GetLength();
  pos.Normalize();
  pos *= m_renderDistance;

  if(m_frame == "sun")
  {
    // If this is the sun, set the rotation to describe the accompanying
    // directional light's direction. This assumes that light will have its
    // direction set to (1, 0, 0) in the .sdf file.
    const double azimuth = atan2(-pos[1], -pos[0]);
    const double zenith = atan2(pos[2], sqrt(pos[0] * pos[0] + pos[1] * pos[1]));
    m_model->SetWorldPose(math::Pose(pos[0], pos[1], pos[2], 0, zenith, azimuth));
  }
  else
  {
    // Otherwise, set the rotation as expected, so any texture map on the body
    // will be properly oriented.
    geometry_msgs::Quaternion& rot = sXform.transform.rotation;
    math::Quaternion quat(rot.w, rot.x, rot.y, rot.z);
    m_model->SetWorldPose(math::Pose(pos, quat));
  }

  const double scale = m_radius * m_renderDistance / distance;
  m_model->SetScale(ignition::math::Vector3d(scale, scale, scale), true);
}

