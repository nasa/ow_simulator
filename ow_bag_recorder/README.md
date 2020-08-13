The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

ow_bag_recorder
===============
Creates a node called bag_recorder_node, that wraps the rosbag recorder API. 
Rosbag recorder options and topics to be recorded can be speicified by a two
different configuration files located in this package's config directory titled
options.yaml and topics.yaml respectively. 

This node can be used in a launch file using the format:
```XML
<node name="bag_recorder_node" pkg="ow_bag_recorder" 
      type="bag_recorder_node" output="screen">
  <rosparam file="$(find ow_bag_recorder)/config/options.yaml"/>
  <rosparam file="$(find ow_bag_recorder)/config/topics.yaml"/>
</node>
```

Bag Files
---------
Recorded bag files wind up in your `~/.ros/` directory or where ever your
`$ROS_HOME` enivronment variable is set to.

options.yaml
------------
Located in the config directory is options.yaml which allows the user to specify
the following options to the rosbag API.

| Option        | Default Value  | Description |
|---------------|----------------|-------------|
| name          | ''             | Record bag file to NAME.bag (crashes when assigned). |
| node          | ''             | Record all topics subscribed to by a specific node. |
| prefix        | ''             | String that's prefixed to the bag file. |
| append_date   | true           | Append a data to bag file name. |
| publish       | false          | Publish a message when the recording begins. |
| verbose       | false          | Verbose output from rosbag API. |
| quiet         | false          | Suppress console output. |
| record_all    | false          | Record all topics. WARNING: OceanWATERS produces a lot of data. This option will produce Gb large bag files if OW runs for only a minute. |
| regex         | false          | Interpret the contents of topics.yaml as regex expressions. |
| exclude_regex | ''             | Exclude topics matching the pattern (subtracts from -a or regex). |
| split         | false          | Split bag file when the maximum size or duration is reached. |
| max_splits    | 0              | Maximum number of bag file splits (0 = infinite) |
| max_size      | 0              | (in MB) Maximum size of a bag file (0 = infinite) |
| max_duration  | -1.0           | (in seconds) Maximum duration of bag file (-1.0 = infinite). |
| limit         | 0              | Maximum number of messages recorded on each topic (0 = infinite). |
| compression   | 'uncompressed' | Compression format of the bag file. Can be "uncompressed", "bz2", or "lz4". |
| buffer_size   | 256            | (in MB) Changes the internal buffer size (0 = infinite). |
| chunk_size    | 768            | (in kB) Advanced. Changes size of recorded chunks. |

topics.yaml
-----------
Located in the config directory is topics.yaml which allows the user to specify
which ROS topics to record. Topics should be specified using YAML collection
format. For instance
```yaml
topics:
  - /rosout
  - /joint_states
```
will record all messages published on `/rosout` and `/joint_states` topics.

If the regex option is set to true, this file can also use regex expression to
specify topics. For instance
```yaml
topics:
  - '\/StereoCamera\/.*\/image_.*'
```
will record `image_color`, `image_mono`, `image_raw`, `image_rect`,
`image_rect_color`, and `image_trigger` from both the `/StereoCamera/left` and
`/StereoCamera/right` topics. 
**NOTE:** When using regex in a YAML file, one must wrap the pattern expression
in single-quotations.