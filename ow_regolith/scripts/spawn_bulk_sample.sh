#!/bin/bash
# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

USAGE="./spawn_bulk_sample RSDF_FILE DIAMETER TOTAL_MASS [-t|--test]
    Generates an SDF model that is spawned some number of times the into the
    Gazebo sim at a point just above the scoop's opening. The number of models
    spawned will have a mass that sums to as near to the TOTAL_MASS as possible.
    The provided RSDF_FILE should be in the Embedded Ruby format, and should 
    accept a diameter argument, which is assigned the value of DIAMETER. The 
    bulk material properties of the model should be specified in the RSDF_FILE.
    
    Caveats:
    1) Before running this script ensure the Gazebo sim is not paused. Models
       are spawned one at a time into the sim to avoid collisions, but that only 
       works if they are able to fall after they spawn.
    2) Spawning a large number of models takes a long time, so it is always good
       practice to use the the -t option flag on a set of arguments to see how 
       many models will result from them before removing the -t option and 
       running the command normally.

    RSDF_FILE   An Embedded Ruby SDF file that accepts a diameter argument.
    DIAMETER    The diameter of the resulting the SDF model. If RDSF_FILE 
                specifies a shape besides a sphere, this can tought of as an 
                approximate model size.
    TOTAL_MASS  The target total mass of all models that will be spawned.
    -t, --test  Calculate and print relevant values, but stop short of spawning
                models into the Gazebo."

if [[ "$#" -ne 3 ]] && [[ "$#" -ne 4 ]]; then
    echo "Incorrect number of arguments!"
    echo "$USAGE"
    exit 1
fi

RSDF_FILE=$1
DIAMETER=$2
TARGET_MASS=$3

# time between each model spawns
# NOTE: This waits on system time, not Gazebo time, so if you see models 
#       spawning over each other, then this value may need to be made larger
SPAWN_WAIT=0.05 # seconds
SPAWN_OFFSET="-z -0.05"
SPAWN_REFERENCE_FRAME="lander::l_scoop_tip"

GENERATED_SDF=`erb diameter=$DIAMETER $RSDF_FILE`

# grab model mass from the newly generated SDF
MASS_PER_MODEL=`echo $GENERATED_SDF | grep -oP '<mass>\s*\K\d+\.?\d*(?=\s*</mass>)'`

SPAWN_NUM=`awk -v M=$TARGET_MASS -v m=$MASS_PER_MODEL 'BEGIN {
    N=M/m;
    printf("%.0f\n", N);
}'`

ACTUAL_TOTAL_MASS=`awk -v m=$MASS_PER_MODEL -v N=$SPAWN_NUM 'BEGIN {
    aM=m*N;
    printf("%f\n", aM);
}'`

MASS_DEFICIT=`awk -v aM=$ACTUAL_TOTAL_MASS -v M=$TARGET_MASS 'BEGIN {
    dM=aM-M;
    printf("%f\n", dM);
}'`

echo "Bulk material properties specified from ................. $RSDF_FILE"
echo "Target mass is .......................................... $TARGET_MASS kg"
echo "Model diameter (size) is ................................ $DIAMETER m"
echo "Mass per model is ....................................... $MASS_PER_MODEL kg"
echo "Total models that will spawn is ......................... $SPAWN_NUM"
echo "Totaling to a mass of ................................... $ACTUAL_TOTAL_MASS kg"
echo "This total mass value differs from the target mass by ... $MASS_DEFICIT kg"

if [ "$4" == "-t" ] || [ "$4" == "--test" ]; then
    echo "This is just a test. Skipping spawn step."
    exit 0
fi

echo "Spawning models into Gazebo world one at a time..."

for (( i=0; i<$SPAWN_NUM; i++ ))
do
    NOW=`date +%s%3N`
    echo $GENERATED_SDF | rosrun gazebo_ros spawn_model -sdf -stdin -reference_frame $SPAWN_REFERENCE_FRAME $SPAWN_OFFSET -model regolith_$NOW;
    # allow previous model to fall before spawning the next one
    sleep $SPAWN_WAIT
done

echo "All models have been spawned"

exit 0
