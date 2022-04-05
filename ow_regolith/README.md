The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

# ow_regolith

* [Introduction](#introduction)
* [Dependencies](#dependencies)
* [How it Works](#how-it-works)
* [Caveats](#caveats)
* [Usage](#usage)
  - [Launch File](#launch-file)
  - [ROS Service](#ros-service)
* [Generating Custom Regolith Particles](#generating-custom-regolith-models)
  - [Adding Models to Gazebo Model Database](#adding-models-to-gazebo-model-database)

## Introduction

This package creates node called `regolith_node` that responds to modifications 
made to the visual terrain model by spawning a model in the scoop to simulate 
the collection of material from the terrain. 

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
tracked total. When that tracked total volume displaced reaches a threshold, a 
regolith model is spawned in the approximate location the terrain was changed.
The threshold is then deducted from the tracked volume.

Spawning is done by calling the `/gazebo/spawn_sdf_model` ROS service, followed
by a call to `/gazebo/apply_body_wrench`, so that the model is kept in the scoop
with a *fake force* of a magnitude that's just enough to keep the particle from 
rolling out. Upon completion of a dig action, the *fake force* is removed from
all regolith particles, so they may settle within the scoop and behave like
normal rigid-bodies during arm movement.

When a regolith particles is dropped and collides with the terrain, the node
removes it from the Gazebo world, and logs a message.

## Caveats

- The *fake force* applied to particles during any dig operation will be 
transferred to the joints of the arm, but the magnitude and direction of the 
force itself is not necessarily realistic to what a digging scoop may 
experience. Keep this in mind when using this plug-in for any study that records
forces on the arm.
- ROS services arm activities are not supported by this node, and it will not 
work properly if the user calls either the dig linear, dig circular, or delivery
ROS service.
- While sample is in the scoop you may see the following error flood the console
```ODE Message 3: LCP internal error, s <= 0 (s=0.0000e+00)```
We're still looking into how to avoid or suppress this message. For now it just
has to be ignored.

## Usage

### Launch File

The `regolith_node` may be added to a launch file as follows
```xml
<node name="regolith_node" pkg="ow_regolith" type="regolith_node" output="screen">
  <param name="spawn_volume_threshold" type="double" value="1e-3"/>
  <param name="regolith_model_uri"     type="string" value="model://ball_icefrag_2cm"/>
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
- `regolith_model_uri` tells the node which model out of the Gazebo model database
should be spawned each time the `spawn_volume_threshold` is reached.

### ContactSensorPlugin
TODO: fill this out
```xml
<sensor name="contact_sensor" type="contact">
  <contact>
    <collision>collision</collision>
  </contact>
  <plugin name='terrain' filename='libContactSensorPlugin.so'>
    <topic>/ow_regolith/contacts_terrain</topic>
  </plugin>
</sensor>
```

### ROS Services

A minimal interface to `regolith_node` is supported via ROS Services that 
enables debugging and manual handling of specific test cases.

#### Services

- `ow_regolith/spawn_regolith position reference_frame` will spawn a single
regolith model at a `position` relative to `reference_frame`.
- `ow_regolith/remove_all_regolith` will delete all regolith particles from the
Gazebo world.

### Adding Models to Gazebo Model Database
New regolith models have to be added to the Gazebo model database before the
regolith node can spawn them. Do so by following these steps:
1. Make a new folder with a unique name in the `models` directory of this 
package.
2. Save the SDF file as `model.sdf` in the newly created folder
3. Create a `model.config` file in the same folder. Feel free to simply copy a
`model.config` from one of the other directories in `models` and modify field
entries as needed.
