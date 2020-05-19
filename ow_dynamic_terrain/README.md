# Dynamic Terrain

- [Introduction](#introduction)
- [Requirements](#requirements)
- [Usage](#usage)
- [Demo](#demo)
  - [Modify Terrain with Circle](#modify-terrain-with-circle)
  - [Modify Terrain with Patch](#modify-terrain-with-patch)

## Introduction

A package that adds the capability to update Gazebo terrains dynamically at run-time (both physically and visually).

## Requirements

ROS distro: melodic  
Gazebo version 9.13 or later

> **_NOTE:_** The DynamicTerrainModel plugin relies on a recently merged pull request
https://bitbucket.org/osrf/gazebo/pull-requests/3210/add-setheight-method-to-heightmapshape.
So make sure to link against that latest gazebo9 branch otherwise the plugin wouldn't function as expected.

## Usage

The package is made up of two plugins, namely DynamicTerrainModel (ModelPlugin) and DyanmicTerrainVisual (VisualPlugin).
The two plugins are typically used togther towards a terrain model, although it is possible to use each one separately.

The following excerpt shows how to apply the two plugins towards a DEM object:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model>

    <!-- Insert the ow_dynamic_terrain_model at the model level -->
    <plugin name="ow_dynamic_terrain_model" filename="libow_dynamic_terrain_model.so" />

    <link>
      <collision name="collision">
          <geometry>
          <heightmap>
            <uri>file://media/materials/textures/terrain.png</uri>
            <size>1024 1024 100</size>
            <pos>0 0 0</pos>
          </heightmap>
        </geometry>
      </collision>

      <visual name="terrain-visual">
        <!-- Insert the ow_dynamic_terrain_visual at the visual level -->
        <plugin name="ow_dynamic_terrain_visual" filename="libow_dynamic_terrain_visual.so" />

        <geometry>
          <heightmap>
            <uri>file://media/materials/textures/terrain.png</uri>
            <size>1024 1024 100</size>
            <pos>0 0 0</pos>
          </heightmap>
        </geometry>

      </visual>
    </link>
  </model>
</sdf>
```

## Demo

Launch demo world using `roslaunch ow_dynamic_terrain europa.launch`. Then use one of the two described methods to
modify the terrain

### Modify Terrain with Circle

Then you may perform a terrain operation by submitting a rostopic message as follows:

```bash
rostopic pub /ow_dynamic_terrain/modify_terrain_circle ow_dynamic_terrain/modify_terrain_circle \
  "{operation: lower,
    position: {x: 0,  y: 0},
    outer_radius: 0.1,
    inner_radius: 0.001,
    weight: 1}"
```

Current supported terrain operations are: _raise, lower_. Operations are case-sensitive.

### Modify Terrain with Capsule

Then you may perform a terrain operation by submitting a rostopic message as follows:

```bash
rostopic pub /ow_dynamic_terrain/modify_terrain_capsule ow_dynamic_terrain/modify_terrain_capsule \
  "{operation: lower,
    position1: {x: 0,  y: 0},
    position2: {x: 0,  y: 3},
    outer_radius: 0.04,
    inner_radius: 0.02,
    weight: 1}"
```

Current supported terrain operations are: _raise, lower_. Operations are case-sensitive.


### Modify Terrain with Patch

```bash
rosrun ow_dynamic_terrain modify_terrain_patch_pub.py image_file
```

> **_NOTE:_** Only single channel 32/64 float image formats are supported.