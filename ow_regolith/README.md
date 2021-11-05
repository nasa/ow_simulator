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
* [Generating Custom Regolith Models](#generating-custom-regolith-models)
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
regolith model is spawned in the scoop and the tracked total volume has the 
threshold deducted from it.

Spawning is done by calling the `/gazebo/spawn_sdf_model` ROS service, followed
by a call to `/gazebo/apply_body_wrench`, so that the model is kept in the scoop
with a *fake force* of a magnitude that's just enough to keep the particle from 
rolling out. Upon completion of a dig action, the *fake force* is removed from
all regolith models, so they may settle within the scoop and behave like normal
regolith material during any following arm movements.

All regolith models spawned by this node are removed from the Gazebo world upon
completion of the delivery arm action. Future versions of this package will 
incorporate smarter logic around when to clean-up regolith models.

## Caveats

- The fake force applied to particles during any dig operation will be 
transferred to the joints of the arm, but the magnitude of the force itself is 
not necessarily realistic to what a digging scoop may experience. Keep this in 
mind when using this plug-in for any study that records forces on the arm.
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

`spawn_volume_threshold` is the cubic meters that must be removed from the
terrain before a regolith model is spawned. It may have to be fine-tuned if a 
regolith model of larger size is used in place of the default to avoid 
consecutively spawned models colliding with each other.

`regolith_model_uri` tells the node which model out of the Gazebo model database
should be spawned each time the `spawn_volume_threshold` is reached.

### ROS Service

ROS services are not supported by this package at this time.

## Generating Custom Regolith Models

This package contains the directory `rsdf` which has three files that assist
in the creation of custom regolith models. The files contain syntax used by Ruby
Templating, which allows for the easy generation an SDF file similar to the 
default regolith model, `ball_icefrag_2cm`, that this package comes with.

The following three files--`ball_icefrag.rsdf`, `ball_sand.rsdf`, and
`ball_snow.rsdf`--correspond to granular material properties of ice fragments, 
sand, and snow, respectively. The files can also be copied, and their 
parameters modified by the user to create a new custom material.

In order to use an existing RSDF file to create an SDF file the describes a 
sphere of a custom diameter, call the command
```erb diameter=0.001 ball_icefrag.rsdf > model.sdf```
where 0.001 represents a diameter of 1 cm, and `ball_icefrag.rsdf` can be 
swapped out for whichever RSDF file you wish you use. This command will create 
the file `model.sdf` in your working directory. See the next section on how to 
include it in the Gazebo model database, so it can be used by the 
`regolith_node`.

### Adding Models to Gazebo Model Database
Any new models generated have to be added to the Gazebo model database by 
following these steps:
1. Make a new folder with a unique name in the `models` directory of this 
package.
2. Save the SDF file as `model.sdf` in the newly created folder
3. Create a `model.config` file in the same folder. Feel free to simply copy a
`model.config` from one of the other directories in `models` and modify field
entries as needed.