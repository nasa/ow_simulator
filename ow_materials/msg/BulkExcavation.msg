std_msgs/Header header
float64   volume_excavated   # total volume in m^3 represented in this bulk
# parallel arrays that define overall composition of the bulk
uint8[]   material_index          # ID in the material database
float32[] material_concentration  # concentration of material, sums to unity
