The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

# ow_power_system

It creates a node called power_system_node, which monitors power consumption
(mainly based of mechanical power at the current time), publishes the estimated
_state of charge (SOC)_, estimated _remaining useful life (RUL)_ in seconds and
also publishes the estimated _battery temperature_ in degrees Celsius. To
monitor any of these values simply subscribe to their respective topics:

```bash
rostopic echo /power_system_node/state_of_charge
rostopic echo /power_system_node/remaining_useful_life
rostopic echo /power_system_node/battery_temperature
```

state_of_charge (SOC): The SOC estimate provides the amount of energy present in
the battery. It varies from 0 - 100%. When the SOC level goes below a certain
set threshold a fault is detected.

remaining_useful_life (RUL): The RUL estimate is that of the time remaining
before the battery reaches the set SOC threshold.

battery_temperature: The internal battery temperature changes based on the
operating conditions. The battery temperature should not rise above a certain
threshold to prevent the battery from going into thermal runaway.

The RUL, SOC, and battery temperature values are computed via the GSAP
prognostics library based on power input values from a precomputed constant or
variable load csv. The values are published every second.

The predicted RUL and SOC values are expected to fluctuate slightly (rather than
decrease monotonically) due to uncertainties in the Monte Carlo predictor
method, with the predicted value becoming more accurate closer to EOD (end of
discharge). The battery temperature is expected to hover around a constant value
of approximately 20 degrees C.

You may initialize the power_system_node in ros launch file as follows:

```xml
<node name="power_system_node" pkg="ow_power_system" type="power_system_node" />
```

NOTE: Currently the frequency of the three topics is dependent on the update
rate of the mechanical power topic which itself is dependent on the frequency of
joint_states topics. This likely to change.
