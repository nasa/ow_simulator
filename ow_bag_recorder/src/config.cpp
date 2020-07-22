// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "config.h"

#include <ros/ros.h>
#include <ros/exceptions.h>
#include <ros/duration.h>

#include <boost/regex.hpp>

#include <limits>
#include <unordered_map>

static const int KILOBYTES_TO_BYTES = 1024;
static const int MEGABYTES_TO_BYTES = KILOBYTES_TO_BYTES * KILOBYTES_TO_BYTES;

using namespace std;
using namespace rosbag::compression;

static std::unordered_map<std::string, CompressionType> 
  compression_types = {
                        {"uncompressed", Uncompressed},
                        {"bz2",          BZ2},
                        {"lz4",          LZ4}
                      };

template <class T>
static T get_option(const string &key, T defval)
{
  T value;
  ros::param::param(key, value, defval);
  return value;
}

template<>
uint32_t get_option<uint32_t>(const string &key, uint32_t defval)
{
  if (!ros::param::has(key))
    return defval;
  int value;
  ros::param::get(key, value);
  if (value < 0)
    throw ros::InvalidParameterException(key + " not positive or zero");
  return static_cast<uint32_t>(value);
}

template<>
uint64_t get_option<uint64_t>(const string &key, uint64_t defval)
{
  if (!ros::param::has(key))
    return defval;
  int value;
  ros::param::get(key, value);
  if (value < 0)
    throw ros::InvalidParameterException(key + " not positive or zero");
  return static_cast<uint64_t>(value);
}

template<>
CompressionType get_option<CompressionType>(const string &key,
                                            CompressionType defval)
{
  if (!ros::param::has(key))
    return defval;
  string value;
  ros::param::get(key, value);
  // map string to ROS CompressionType enumerate
  auto it = compression_types.find(value);
  if (it == compression_types.end())
    throw ros::InvalidParameterException(key + " is an unrecognized compression type");
  return compression_types[value];
}

template<>
boost::regex get_option<boost::regex>(const string &key, boost::regex defval)
{
  if (!ros::param::has(key))
    return defval;
  string value;
  ros::param::get(key, value);
  return boost::regex(value);
}

template<>
ros::Duration get_option<ros::Duration>(const string &key, ros::Duration defval)
{
  if (!ros::param::has(key))
    return defval;
  double value;
  ros::param::get(key, value);
  return ros::Duration(value);
}

bool get_recorder_options(rosbag::RecorderOptions &out_options)
{
  try
  {   
    out_options.name          = get_option("/bag_recorder_node/options/name",          out_options.name);
    out_options.node          = get_option("/bag_recorder_node/options/node",          out_options.node);
    out_options.prefix        = get_option("/bag_recorder_node/options/prefix",        out_options.prefix);
    out_options.append_date   = get_option("/bag_recorder_node/options/append_date",   out_options.append_date);
    out_options.publish       = get_option("/bag_recorder_node/options/publish",       out_options.publish);
    out_options.verbose       = get_option("/bag_recorder_node/options/verbose",       out_options.verbose);
    out_options.quiet         = get_option("/bag_recorder_node/options/quiet",         out_options.quiet);
    out_options.record_all    = get_option("/bag_recorder_node/options/record_all",    out_options.record_all);
    out_options.regex         = get_option("/bag_recorder_node/options/regex",         out_options.regex);
    out_options.exclude_regex = get_option("/bag_recorder_node/options/exclude_regex", out_options.exclude_regex);
    out_options.split         = get_option("/bag_recorder_node/options/split",         out_options.split);
    out_options.max_splits    = get_option("/bag_recorder_node/options/max_splits",    out_options.max_splits);
    out_options.max_size      = get_option("/bag_recorder_node/options/max_size",      out_options.max_size);
    out_options.max_duration  = get_option("/bag_recorder_node/options/max_duration",  out_options.max_duration);
    out_options.limit         = get_option("/bag_recorder_node/options/limit",         out_options.limit);
    out_options.compression   = get_option("/bag_recorder_node/options/compression",   out_options.compression);
    
    // We accept megabytes for buffer_size and kilobytes for chunk_size in the 
    // YAML, but RecorderOptions uses bytes for both, so both have to be 
    // converted to bytes
    out_options.buffer_size = MEGABYTES_TO_BYTES 
        * get_option("/bag_recorder_node/options/buffer_size", out_options.buffer_size / MEGABYTES_TO_BYTES);
    out_options.chunk_size  = KILOBYTES_TO_BYTES 
        * get_option("/bag_recorder_node/options/chunk_size",  out_options.chunk_size / KILOBYTES_TO_BYTES);
    
    // the following options exist in RecorderOptions but either their usage is
    // not documented or they are unlikley to be needed by our userbase, so we 
    // are not officially supporting them
    // out_options.trigger       = get_option("/bag_recorder_node/options/trigger",       out_options.trigger);
    // out_options.snapshot      = get_option("/bag_recorder_node/options/snapshot",      out_options.snapshot);
    // out_options.min_space_str = get_option("/bag_recorder_node/options/min_space_str", out_options.min_space_str);
    // out_options.min_space     = get_option("/bag_recorder_node/options/min_space",     out_options.min_space);
  }
  catch (const ros::InvalidParameterException &except) 
  {
    ROS_ERROR("%s", except.what());
    return false;
  }

  // logic for options that aren't user-facing
  out_options.do_exclude = !out_options.exclude_regex.empty();

  return true;
}

void get_recorder_topics(std::vector<std::string> &out_topics)
{
  out_topics = get_option("/bag_recorder_node/topics", out_topics);
}