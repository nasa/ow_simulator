<h4>This the the README file for the ow_power_system package.</h4>


It creates a node called power_system_node, which listens for power consumption in a topic called power_draw, and publishes the estimated state of charge (SOC) in a topic called state_of_charge.

Currently, the state_of_charge topic publishes 0.0 every one second. This is a placeholder until the node is linked up to the electrochemistry-based battery model in GSAP (the Generalized Software Architecture for Prognostics). 

This node can be launched using the format
`<node name="power_system_node" pkg="ow_power_system" type="power_system_node" args=""/>`

It should be launched in common.launch (roslaunch ow common.launch), although it does not do so for Build 4.

It may eventually have its own launch files to ease implementation of different battery configurations, but for now, we find the use of common.launch to be sufficient.

