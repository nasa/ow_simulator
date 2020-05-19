OceanWATERS Gazebo plugins
==================================
celestial_body_plugin
-------------------
This is a model plugin that can only be use within a Gazebo `<model>` tag like this:

```
<?xml version="1.0"?>
<sdf version='1.6'>
    
  <model name="ow_sun">
    <!-- This pose will be overridden by celestial_body_plugin. -->
    <pose>0 0 0  0 0 0</pose>
    <static>0</static>
    <link name='ellipsoid'>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <sphere>
            <radius>1</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <uri>model://ow_sun/materials/scripts</uri>
            <name>ow/sun</name>
          </script>
        </material>
      </visual>
      <light name='sun' type='directional'>
        <cast_shadows>1</cast_shadows>
        <direction>1 0 0</direction>
        <diffuse>1 1 1 1</diffuse>
        <specular>1 1 1 1</specular>
      </light>
    </link>
    <plugin name="celestial_body" filename="libcelestial_body_plugin.so">
      <frame>sun</frame>
      <radius>696392000</radius>
      <render_distance>160000</render_distance>
    </plugin>
  </model>

</sdf>
```

#### XML tags
 - `<frame>` - Specify the frame to lookup in the tf-tree and apply to this model.
 - `<radius>` - Specify the actual radius of this body, the same radius used in your model file.
 - `<render_distance>` - Distance from origin to render this body. Use a value within your far clipping plane.

#### Explanation
This plugin can be used to position a model in the sky using a given frame.
The given frame is looked up relative to `site`. This will affect the rotation
of a body and its position relative to Gazebo's origin.

If the given frame is `sun`, its rotation will be set to affect the light
direction instead of affecting the sun's rotation. The direction of the light
source in the .sdf file must be (1, 0, 0).

