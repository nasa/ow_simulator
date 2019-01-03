OceanWATERS Gazebo plugins
==================================
irradiance_map_plugin
-------------------
This plugin renders a dynamic cubemap and processes it to create an irradiance
environment map.

This is a visual plugin that can only be use within a Gazebo `<visual>` tag like this:
```
<visual name="visual">
  .
  .
  .
  <plugin name="irradiance_map" filename="libirradiance_map_plugin.so">
    <texture_unit>ow_irradiance_environment_map</texture_unit>
  </plugin>
</visual>
```

Your irradiance map can be used globally. Wherever you want it to be used, add a
texture_unit like this to an Ogre3D material script:
```
  texture_unit ow_irradiance_environment_map
  {
  }
```
