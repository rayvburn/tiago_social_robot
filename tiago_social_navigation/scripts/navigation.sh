#! /bin/sh
#
# Check if the $HOME/.pal folder is available and creates it if needed before
# launching navigation.
#
# Usage: $0 <robot> [<state>] [<localization method>] [<mapping method>] [<map>]

# Check parameters:
if [ $# -lt 1 ]; then
  echo "Usage: $0 <robot> [<state>] [<localization method>] [<mapping method>] [<map>] [<octomap>]"
  echo "Check if the $HOME/.pal folder is availabe and creates it if needed"
  echo "before launching navigation."
  exit 1
else
  ROBOT=$1
fi

skip_map_operations="0"

echo "[tiago_social_navigation/navigation.sh] Starting tiago_social_navigation script. Arguments are: "
for i in $*; do 
  echo "\t$i" 
done

if [ $# -lt 2 ]; then
  STATE=localization
else
  STATE=$2
fi

if [ $# -lt 3 ]; then
  LOCALIZATION=amcl
else
  LOCALIZATION=$3
fi

if [ $# -lt 4 ]; then
  MAPPING=gmapping
else
  MAPPING=$4
fi

if [ $# -lt 5 ]; then
  MAP=$HOME/.pal/${ROBOT}_maps/config
elif [ "$5" = "none" ]; then
  skip_map_operations="1"
  echo "[tiago_social_navigation/navigation.sh] Skipping map operations" 
  MAP="none"
else
  MAP=$5
fi

#if [ -d "$MAP" -a "$STATE" = "mapping" ]; then
#  rm -rf $MAP
#fi

if [ $# -lt 7 ]; then
  MULTI="false"
  ROBOT_NAMESPACE=""
else 
  if [ "$7" = "true" ]; then
    if [ $# -lt 8 ]; then
      echo "[tiago_social_navigation/navigation.sh] If MULTI is true I need the robot_namespace"
      exit 1
    else
      MULTI="true"
      ROBOT_NAMESPACE=$8
    fi
  else 
    MULTI="false"
    ROBOT_NAMESPACE=""
  fi
fi

if [ $# -lt 11 ]; then
  MAP_TOPIC="map"
else
  MAP_TOPIC=$11
fi

if [ $# -lt 12 ]; then
  if [ ! -f "$HOME/.pal/pose.yaml" ]; then
    echo "[tiago_social_navigation/navigation.sh] '$HOME/.pal/pose.yaml' not found and will be created"
    rosrun pal_navigation_sm cp_pose_to_home.sh
  fi
  LOCALIZATION_POSE_ESTIMATE="$HOME/.pal/pose.yaml"
else
  LOCALIZATION_POSE_ESTIMATE=$12
fi

if [ $# -lt 13 ]; then
  echo "[tiago_social_navigation/navigation.sh] You have to provide a path to configuration file for the localization component, use 'gazebo_nav_base.launch'"
  exit 3
else
  LOCALIZATION_CONFIG_FILE=$13
fi

if [ $# -lt 14 ]; then
  echo "[tiago_social_navigation/navigation.sh] You have to provide a path to configuration file for the mapping component, use 'gazebo_nav_base.launch'"
  exit 4
else
  MAPPING_CONFIG_FILE=$14
fi

# Ensure target file exists
if [ "$skip_map_operations" = "0" ]; then
    if [ ! -f "$MAP" ]; then
        echo "[tiago_social_navigation/navigation.sh] Error: Cannot find path: $MAP"
        exit 5
    fi
fi

if [ "$STATE" = "localization" ]; then
  CONFIGURATION_FILE=$LOCALIZATION_CONFIG_FILE
elif [ "$STATE" = "mapping" ]; then
  CONFIGURATION_FILE=$MAPPING_CONFIG_FILE
else
  echo "[tiago_social_navigation/navigation.sh] Error: Cannot select configuration file because wrong STATE was given ($STATE)"
  exit 6
fi

# Run localization/mapping
echo "[tiago_social_navigation/navigation.sh] Invoking: 'roslaunch ${ROBOT}_social_navigation $STATE.launch localization:=$LOCALIZATION mapping:=$MAPPING map:=$MAP multiple:=$MULTI robot_namespace:=$ROBOT_NAMESPACE map_topic:=$MAP_TOPIC pose_estimate_file:=$LOCALIZATION_POSE_ESTIMATE config_file:=$CONFIGURATION_FILE'"
roslaunch ${ROBOT}_social_navigation $STATE.launch localization:=$LOCALIZATION mapping:=$MAPPING map:=$MAP multiple:=$MULTI robot_namespace:=$ROBOT_NAMESPACE map_topic:=$MAP_TOPIC pose_estimate_file:=$LOCALIZATION_POSE_ESTIMATE config_file:=$CONFIGURATION_FILE
