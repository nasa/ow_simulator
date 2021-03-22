The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

ow_power_system
===============
It creates a node called power_system_node, which listens for power consumption
in a topic called power_draw, publishes the estimated state of charge (SOC)
in a topic called state_of_charge, publishes the estimated remaining useful life (RUL) in seconds in a topic called
remaining_useful_life, and publishes the estimated battery temperature in degrees Celsius in a topic called 
battery_temperature.

The RUL, SOC, and battery temperature values are computed via the GSAP prognostics library based on power input
values from a precomputed constant or variable load csv. The values are published every second.

The predicted RUL and SOC values are expected to fluctuate slightly (rather than decrease monotonically) due to 
uncertainties in the Monte Carlo predictor method, with the predicted value becoming more accurate closer to EOD 
(end of discharge). The battery temperature is expected to hover around a constant value of approximately 20 deg. C.

Once the publisher hits the end of the csv, it will repeat the dissipation sequence from the top.

This node can be launched using the format:
```xml
<arg name="power_draw_csv_file" default="data_const_load.csv"/>  <!-- options: data_const_load.csv, data_variable_load.csv -->

<node name="power_system_node" pkg="ow_power_system" type="power_system_node">
    <param name="power_draw_csv_path" value="$(find ow_power_system)/data/$(arg power_draw_csv_file)"/>
</node>
```

There are two options for the power draw: either constant load or variable load, with the specs listed below.
  
**SPECS:**  
data_const_load.csv  
Voltage range: 4.10V to 3.21V  
Update interval: 1 second  
Useful life: 2738s  
  
data_variable_load.csv  
Voltage range: 4.10V to 3.20V  
Update interval: 1 second  
Useful life: 4067s  
