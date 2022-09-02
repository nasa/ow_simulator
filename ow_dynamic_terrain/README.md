The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

# Dynamic Terrain

* [Introduction](#introduction)
* [Requirements](#requirements)
  - [Compatibility](#compatibility)
* [Usage](#usage)
  - [Control Visual and Physical Aspects of the Terrain Individually](#control-visual-and-physical-aspects-of-the-terrain-individually)
* [Demo](#demo)
  - [Modify Terrain with Circle](#modify-terrain-with-circle)
  - [Modify Terrain with Ellipse](#modify-terrain-with-ellipse)
  - [Modify Terrain with Patch](#modify-terrain-with-patch)

## Introduction

A package that adds the capability to update Gazebo terrains dynamically at run-time (both physically and visually).

## Requirements

* This package was developed and tested against `ROS melodic` distrbution.
* One of the plugins included in this package _DynamicTerrainModel_ requires `Gazebo` version 9.13 or later.

### Compatibility
* The _IRGLinkTracksPlugin_ and _DynamicTerrainVisual_ plugin may conflict with each other as both affect the visual
appearance of the terrain.
* The _DynamicTerrainVisual_  is incompatible with the _HeightmapLODPlugin_; Either disable the plugin or configure it
to always use the highest LOD level.

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

`libow_dynamic_terrain_visual.so` can apply an Ogre-generated normal map upon
initialization and will update that normal map each time the terrain is
modified. If you are using a custom Ogre material script, get this normal map
by adding the line:

```
texture_unit ow_dynamic_terrain_normal_map {}
```

You can specify a texture map file in a texture unit, but there is no reason to
do so when using the above texture unit name as it will be overridden by the
plugin.

Once these plugins have been configured for a heightmap object, the user can then modify the terrain by composing and
submitting an appropriate ros message to the following three topics/methods:
- */ow_dynamic_terrain/modify_terrain_circle*
- */ow_dynamic_terrain/modify_terrain_ellipse*
- */ow_dynamic_terrain/modify_terrain_patch*

See [Demo](#demo) for additional details on each method.

## Control Visual and Physical Aspects of the Terrain Individually

In certain situations it may be desirable to only modify one of the two terrain aspects: the visual part and the
physical. This certainly can be achieved by enabling one of the two plugins for the model. However, in other cases the
user may want to alter each aspect slightly different than the other. To handle such use case, the two plugins
_DynamicTerrainVisual_ and _DynamicTerrainModel_ both add an extension to the three topics listed above that correspond
to their aspect. i.e. The _DynamicTerrainVisual_ plugin has the following three additional topics that it subscribes to:
- */ow_dynamic_terrain/modify_terrain_circle/visual*
- */ow_dynamic_terrain/modify_terrain_ellipse/visual*
- */ow_dynamic_terrain/modify_terrain_patch/visual*

Meanwhile, the _DynamicTerrainModel_ plugin adds the following three topics:
- */ow_dynamic_terrain/modify_terrain_circle/collision*
- */ow_dynamic_terrain/modify_terrain_ellipse/collision*
- */ow_dynamic_terrain/modify_terrain_patch/collision*

## Demo

Launch demo world using `roslaunch ow_dynamic_terrain europa.launch`. Then use one of the two described methods to
modify the terrain

### Modify Terrain with Circle

You may modify the terrain with a circle shaped modify operation by publishing a message as follows:

```bash
rostopic pub --once /ow_dynamic_terrain/modify_terrain_circle ow_dynamic_terrain/modify_terrain_circle \
  "{position: {x: 0,  y: 0, z: 2},
    outer_radius: 0.1,
    inner_radius: 0.001,
    weight: -1.0,
    merge_method: 'min' }"
```

The listing below explains the message parameters:

* *position*: is in reference to gazebo world coordinates.
  * The x and y values of the position define the center of the circle.
  * The z - if used - would offset all values generated by the modify operation. i.e the generated height value is
  equivalent to (z + effective_weight).
* *outer_radius*: The hard limit of the operation, beyond that the operation has no effect.
* *inner_radius*: The inner radius is where the provided weight is full realized, beyond the inner radius the weight
fades down up to the outer distance at a quadratic rate. inner_radius value may not exceed outer_radius.
* *weight*: Depending on the weight sign the operation will either raise or lower the terrain around the provided
 position.
* *merge_method*: The merge_method parameter decides how to merge generated values with height values of the terrain; 
available choices:
  * *keep*: current_height_value is left unchanged.
  * *replace*: current_height_value is replaced by generated_height_value.
  * *add*: The new height would be the sum of current_height_value and generated_height_value.
  * *sub*: The generated_height_value is subtracted from current_height_value.
  * *min*: The generated_height_value would replace current_height_value iff generated_height_value < current_height_value.
  * *max*: The generated_height_value would replace current_height_value iff generated_height_value > current_height_value.
  * *avg*: The new height value would be the average of current_height_value and generated_height_value.
  * *default*: If no merge_method is specified then the default merge_method is add.
  
In most scenarios, you would want to set position.z parameter to zero when using either 'add' and 'sub' as a merge_method
 such that additions and subtractions and subtractions are applied as offsets to the current height of the terrain.
 Meanwhile, for 'min', 'max' and 'avg' you would want to set the z value of the position parameter to match the elevation
 of where you want to have the generated hemisphere position which will eventually intersect at that level with the terrain.

In the above example, with z value set to 2.0 and weight is -1.0 this would produce a hemisphere with lowest value of 
2.0 + (-1.0) = 1.0. Since merged_method is set to 'min', if current height values of the terrain at point (x = 0, y = 0)
exceed 1.0 will be replaced by 1.0.

### Modify Terrain with Ellipse

You may perform an ellipse modify terrain operation by publishing a message as follows:

```bash
rostopic pub --once /ow_dynamic_terrain/modify_terrain_ellipse ow_dynamic_terrain/modify_terrain_ellipse \
  "{position: {x: 0,  y: 0, z: 2},
    orientation: 30.0,
    outer_radius_a: 0.2,
    outer_radius_b: 0.1,
    inner_radius_a: 0.001,
    inner_radius_b: 0.001,
    weight: -1.0,
    merge_method: 'min'}"
```

The parameters used here are mostly the same as the ones described in [Modify Terrain with Circle](#modify-terrain-with-circle)
 section with the exception of *outer_radius_a*, *outer_radius_b*, *inner_radius_a*, *inner_radius_b*. These parameters
 correspond to the two radii of an ellipse (a, b). The weight is applied at 100% within the inner ellipse, 0% outside the
 outer ellipse, In the distance between the outer ellipse and inner one, the applied weight fades out quadratically.  

Additionally, an ellipse can have an orientation parameter (measured in degrees) that would rotate the ellipse around
 its center.

### Modify Terrain with Patch

You may use the provide modify_terrain_patch helper utility to submit a tiff image to modify the terrain as shown in the example below:

```bash
roscd ow_dynamic_terrain
rosrun ow_dynamic_terrain modify_terrain_patch_pub.py misc/first_pass_heightmap.tif
```
The example adds the supplied first_pass_heightmap.tif example to the loaded terrain around the origin point.  
  
The following example applies only the height values of the image that exceeds the current elevation values of the terrain
 which for the europa demo sets at ~1.5 near the center:
```bash
roscd ow_dynamic_terrain
rosrun ow_dynamic_terrain modify_terrain_patch_pub.py --position_z 1.5 --merge_method max misc/first_pass_heightmap.tif
```

Full options of the modify_terrain_patch_pub program:
```bash
rosrun ow_dynamic_terrain modify_terrain_patch_pub.py [-h]
                                   [--position_x POSITION_X]
                                   [--position_y POSITION_Y]
                                   [--position_z POSITION_Z]
                                   [--orientation ORIENTATION]
                                   [--z_scale Z_SCALE]
                                   [--merge_method MERGE_METHOD]
                                   image
```

The listing below explains used parameter:
* *image* (required): accepts a path to a valid image file.
* *position*: maps the center of the image to the terrain, given in gazebo world coordinates.
* *orientation*: rotate the image around its center; measured in degrees.
* *z_scale*: a scale factor that will be applied to image intensities. If omitted or zero is specified then the image
 intensities will be normalized.
* *merge_method*: decides how to merge the height values of supplied image with the current height values of the terrain.
The choices are the same ones listed in the [Modify Terrain with Circle](#modify-terrain-with-circle) section.

> **_NOTE:_** Currently only single channel 32 float image formats are supported.

You may refer to the project [wiki](https://github.com/nasa/ow_simulator/wiki) for more details.
