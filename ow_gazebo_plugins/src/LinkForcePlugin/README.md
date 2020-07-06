The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

OceanWATERS Gazebo plugins
==================================
LinkForcePlugin
-------------------
This is a model plugin that can only be use within a Gazebo `<model>` tag like this:

```
<robot>
  .
  .
  <link name="l_scoop">
  .
  .
  <gazebo>
    .
    .
    .
    <plugin name="LinkForce" filename="libLinkForcePlugin.so">
      <link>l_scoop</link>
      <lookupTable>$(find ow_gazebo_plugins)/data/scoop_force_circular.csv</lookupTable>
    </plugin>
  </gazebo>
</robot>
```

#### XML tags
 - `<link>` - Specify the link that forces will act upon.
 - `<lookupTable>` - Specify the lookup table to use.

#### Explanation
This plugin is currently narrowly scoped for the task of applying forces from
the scoop_force_circular.csv lookup table. It is not flexible enough to handle
other lookup tables or other methods of applying forces.

Gazebo is not capable of switching plugins at runtime, so if we want to
apply forces by other methods or from other lookup tables, this plugin will
require refactoring either to perform other tasks or to be disabled at runtime
so other plugins can apply forces.

