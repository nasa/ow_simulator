The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

OceanWATERS Gazebo plugins
==================================
CosimulatorPlugin
-------------------
This is a model plugin that can be use within a `<gazebo>` tag like this:

```
<?xml version="1.0"?>
<robot>

  ...

  <gazebo>

    ...

    <plugin name="CosimulationPlugin" filename="libCosimulationPlugin.so">
      <!-- all parameters given in SI units -->
      <!-- link_ and particle_ are set to the same values used in EDEM sim -->
      <link>l_scoop</link>
      <link_model>l_scoop.STL</link_model>
      
      <link_youngs_modulus>6.9e10</link_youngs_modulus>
      <link_poisson_ratio>0.33</link_poisson_ratio>
      <link_restitution_coef>0.7</link_restitution_coef>
      <link_static_friction_coef>0.58</link_static_friction_coef>
      <link_rolling_friction_coef>0.05</link_rolling_friction_coef>

      <particle_youngs_modulus>2.72e7</particle_youngs_modulus>
      <particle_poisson_ratio>0.36</particle_poisson_ratio>
      <particle_restitution_coef>0.88</particle_restitution_coef>
      <particle_static_friction_coef>0.58</particle_static_friction_coef>
      <particle_rolling_friction_coef>0.05</particle_rolling_friction_coef>
      <particle_density>300</particle_density>
      <particle_radius>1.75e-3</particle_radius>

      <workspace_dimensions>0.5 0.5 0.2</workspace_dimensions>
      <workspace_pose>0 0 0 1 0 0 0</workspace_pose>

      <timestep>1e-6</timestep>
    </plugin>
  </gazebo>
</robot>
```

#### XML tags
 - `<link>` - Link in model that will be cosimulated. Forces and torques that
 are calculated in the cosimulation are applied to this link.
 - `<link_model>` - Path to the collisional model used by `<link>`.
 - `<link_youngs_modulus>` - Youngs modulus of `<link>` in GPa.
 - `<link_poisson_ratio>` - Poisson ratio of `<link>`.
 - `<link_restituion_coef>` - Restitution coefficient of `<link>`.
 - `<link_static_friction>` - Coefficient of static friction between `<link>` 
 and cosimulation particles. 
 - `<link_rolling_friction>` - Coefficient of rolling friction between `<link>`
 and cosimulation particles.
 - `<particle_youngs_modulus>` - Bulk youngs modulus of particles in GPa.
 - `<particle_poisson_ratio>` - Bulk poisson ratio of particles.
 - `<particle_restitution_coef>` - Restitution coefficient of particles.
 - `<particle_static_friction>` - Coefficient of static friction between
 particles.
 - `<particle_rolling_friction>` - Coefficient of rolling friction between
 particles.
 - `<particle_density>` - Bulk density of particles.
 - `<particle_radius>` - Radius of all particles in the cosimulaiton.
 - `<workspace_dimensions>` - Subset region of the Gazebo world inside of which
 the cosimulation takes place. Cosimulation will automatically begin when 
 `<link>` enters this region.
 - `<workspace_pose>` - Pose of the workspace.

#### Explanation
This plugin can be used to estimate forces on a link that interacts with a
granular medium, such as the ground, using Discrete Element Method (DEM). At the
start of each Gazebo physics loop, Gazebo will provide the plugin with the pose 
of `<link>`, the cosimulation will then compute and provide Gazebo with the 
force and torques on the `<link>` resulting from its pose. 

Workspace is a region that should intersect with the terrain being simulated
in the approximate area where `<link>` is expected to interact. Since DEM is
typically highly computationally demanding, running this plugin in realtime is
impossible, and significant computation (anywhere between days or hours) should
be expected. Computation time can be improved by minimizing the size of
workspace, which gives the cosimulation fewer discrete elements to simulate. 

#### Development
This plugin is in a stubbed state and will not work properly for its purpose.
When an thirdparty open-source DEM software has been selected for use in
OceanWATERS it will be integrated into this plugin.

