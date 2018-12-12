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
  <plugin name="irradiance_map" filename="libirradiance_map_plugin.so" />
</visual>
```

