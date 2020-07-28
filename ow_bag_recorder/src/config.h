// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef BAG_RECORDER_CONFIG_H
#define BAG_RECORDER_CONFIG_H

// rosbag includes
#include <rosbag/recorder.h>

#include <vector>

// retrieve recorder options from Parameter Server
// use existing settings in out_options as defaults
bool get_recorder_options(rosbag::RecorderOptions &out_options);

// retrieve topics to be recorded from Parameter Server
void get_recorder_topics(std::vector<std::string> &out_topics);

#endif /* BAG_RECORDER_CONFIG_H */