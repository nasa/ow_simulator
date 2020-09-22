// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>
#include <vector>


namespace ow_lander {


typedef enum {
  J_SHOU_YAW,
  J_SHOU_PITCH,
  J_PROX_PITCH,
  J_DIST_PITCH,
  J_HAND_YAW,
  J_SCOOP_YAW,
  J_GRINDER,
  J_ANT_PAN,
  J_ANT_TILT,
  NUM_JOINTS  
} joint_t;

// The following vectors must be ordered exactly the same as the values of joint_t

// Joint names used in lander.xacro. These are often used in ROS topics, such as joint_states
const std::vector<std::string> joint_names {
  "j_shou_yaw", "j_shou_pitch", "j_prox_pitch", "j_dist_pitch", "j_hand_yaw", "j_scoop_yaw",
  "j_grinder", "j_ant_pan", "j_ant_tilt"
};

// These names are good for displaying to humans
const std::vector<std::string> joint_display_names {
  "Shoulder Yaw", "Shoulder Pitch", "Proximal Pitch", "Distal Pitch", "Hand Yaw", "Scoop Yaw",
  "Grinder Yaw", "Antenna Pan", "Antenna Tilt"
};


} // namespace ow_lander

