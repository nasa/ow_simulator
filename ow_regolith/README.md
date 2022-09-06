The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

# ow_regolith

* [Introduction](#introduction)
* [Dependencies](#dependencies)
* [How it Works](#how-it-works)
* [Usage](#usage)
  - [Regolith Node](#regolith-node)
  - [ContactSensorPlugin](#contactsensorplugin)
  - [ROS Services](#ros-services)
  - [Adding Models to the Gazebo Model Database](#adding-models-to-the-gazebo-model-database)
* [Caveats](#caveats)

## Introduction

This package creates a node called `regolith_node` that responds to
modifications made to the visual terrain model by spawning a model in the scoop
to simulate the collection of material from the terrain.

## Dependencies

This package relies on a topic published by the `ow_dynamic_terrain` package. 
The automated spawning of material in the scoop can only occur if that package 
is running.

This package also relies on the `ow_lander` package, namely the result topics 
for dig linear, dig circular, and delivery. It will not work properly if these 
are not available. 

## How it Works

The `regolith_node` subscribes to 
`/ow_dynamic_terrain/modification_differential/visual`, which transports a 
differential image that represents changes in heights that occurred due to tool
modification of the visual terrain model. The `regolith_node` computes the total
volume displaced each time a differential image is published and adds it to a 
tracked total, if it was determined to have been caused by the scoop. When the
tracked total volume displaced reaches a threshold, a regolith model is spawned
in the scoop. The threshold is then deducted from the tracked volume.

Spawning is done by calling the `/gazebo/spawn_sdf_model` ROS service, followed
by a call to `/gazebo/apply_body_wrench`, so that the model is kept in the scoop
with a *fake force* of a magnitude that's just enough to keep the particle from 
rolling out. As the scoop tilts upward, the *fake force* is removed from all
regolith particles, so they may settle within the scoop and behave like normal
rigid-bodies during arm movement.

When a regolith particles is dropped and collides with the terrain, the node
removes it from the Gazebo world.

## Usage

### Regolith Node

The `regolith_node` may be added to any launch file like so
```xml
<node name="regolith_node" pkg="ow_regolith" type="regolith_node" output="screen">
  <param name="regolith_model_uri" type="string" value="model://sphere_2cm"/>
  <param name="spawn_volume_threshold" type="double" value="1e-3"/>
  <param name="spawn_spacing" type="double" value="0.02"/>
</node>
```
The two parameters, `spawn_volume_threshold` and `regolith_model_uri`, are
required by the node, and there will be an error printed if it is ran without
these two being set. 

#### Parameters

- `spawn_volume_threshold` is the cubic meters that must be removed from the
  terrain before a regolith model is spawned. It may have to be fine-tuned if a
  regolith model of larger size is used in place of the default to avoid
  consecutively spawned models colliding with each other.
- `regolith_model_uri` tells the node which model out of the Gazebo model
  database should be spawned each time the `spawn_volume_threshold` is reached.
- `spawn_spacing` (optional) tells the node how far apart successive scoop spawn
  points should be placed horizontal along the scoop opening. If this parameter
  is absent, only one spawn point in the center of the scoop opening will be
  used.Generally it is a good idea to set this value to equal the largest
  dimension of the scoop model you are using.

### ContactSensorPlugin

The ContactSensorPlugin reports links that come into contact with whatever
collision model is assigned to it in the collision parameter of `<contact>`.
This plugin is used to enable the regolith node to remove regolith particles
that come into contact with the terrain. It can be implemented inside of a
`<link>` of an SDF document. Here is an example of how it is implemented in the
Europa worlds.
```xml
<sensor name="contact_sensor_terrain" type="contact">
  <contact>
    <collision>collision</collision>
  </contact>
  <plugin name="contact_sensor_terrain_plugin"
          filename="libContactSensorPlugin.so">
    <topic>/ow_regolith/contacts/terrain</topic>
    <report_only>regolith_\d*.*</report_only>
  </plugin>
</sensor>
```

#### Parameters

- `<topic>` This will be the topic on which the list of link names currently in
  contact with the collision object are published.
- `<report_only>` (optional) A regex pattern. If this parameter is set, then
  only contacts with link names that match the pattern will be reported.

### ROS Services

A minimal interface to `regolith_node` is supported via ROS Services that 
enables debugging and manual handling of specific test cases.

#### Services

- `ow_regolith/spawn_regolith position reference_frame` will spawn a single
  regolith model at a `position` relative to `reference_frame`.
- `ow_regolith/remove_regolith link_names` will either remove any regolith
  particles listed in `link_names`, or if `link_names` is empty, will remove
  all regolith particles present in the world. It's response parameter
  `not_removed` will contain a list of any links that could not be removed.

### Adding Models to the Gazebo Model Database

New regolith models have to be added to the Gazebo model database before the
regolith node can spawn them. Do so by following these steps:
1. Make a new folder with a unique name in the `models` directory of a package.
2. Save the SDF file as `model.sdf` in the newly created folder
3. Create a `model.config` file in the same folder. Feel free to simply copy a
`model.config` from a model folder from another package and modify field entries
as needed.

## Caveats

- The *fake force* applied to particles during any dig operation will be
transferred to the joints of the arm, but the magnitude and direction of the
force itself is not necessarily realistic to what a digging scoop may
experience. Keep this in mind when using this plug-in for any study that records
forces on the arm.
