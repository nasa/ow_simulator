// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef LinkForcePlugin_h
#define LinkForcePlugin_h

#include <gazebo/common/Plugin.hh>
#include "ForceRow.h"


namespace gazebo {

// This plugin is added to a robot description and applies forces from a
// specified lookup table to a specified link in the robot.
class LinkForcePlugin : public ModelPlugin
{
public:
  LinkForcePlugin();
  ~LinkForcePlugin();
    
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  bool LoadLookupTable(std::string filename);

  void OnUpdate();

  bool GetForces(int material, float depth, int pass, float rho,
                 std::vector<float>& out_forces);

  // Find a value in a map when there is not necessarily an exact match for the key
  template <typename key_t, typename value_t>
  value_t FindValueForClosestKey(const std::map<key_t, value_t>& inmap, const key_t& key);

  physics::LinkPtr m_link;

  // Connection to the update event
  event::ConnectionPtr m_updateConnection;

  std::map<int, std::map<float, std::map<int, std::map<float, std::vector<float> > > > > m_forcesMap;
};

}

#endif // LinkForcePlugin_h
