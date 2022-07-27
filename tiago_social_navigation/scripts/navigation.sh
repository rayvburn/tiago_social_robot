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

# Args:
#
# $1   robot_name                             tiago
# $2   $(arg state)                           mapping
# $3   $(arg localization_method)             amcl
# $4   $(arg mapping_method)                  gmapping
# $5   $(arg map_file)                        /home/rayvburn/ros_workspace/ws_social_navigation/src/tiago_sim_integration/maps/012_sim_localization/map.yaml
# $6   $(arg octomap)                         false
# $7   $(arg scan_topic)                      scan_nav
# $8   $(arg laser_model)                     sick-571
# $9   $(arg base_type)                       pmb2
# $10  $(arg multiple)                        false
# $11  $(arg robot_namespace)                 /
# $12  $(arg map_topic)                       map
# $13  $(arg localization_pose_estimate)      /home/rayvburn/ros_workspace/ws_social_navigation/src/tiago_social_robot/tiago_social_navigation/config/localization/pose_estimate.yaml
# $14  $(arg localization_config)             /home/rayvburn/ros_workspace/ws_social_navigation/src/tiago_social_robot/tiago_social_experiments/config/aws_hospital.yaml
# $15  $(arg mapping_config)                  /home/rayvburn/ros_workspace/ws_social_navigation/src/tiago_social_robot/tiago_social_navigation/config/mapping/gmapping.yaml
#

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
  # FIXME: THIS IS NOT A CORRECT FILE!!
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
  SCAN_TOPIC="scan_raw"
else
  SCAN_TOPIC="$7"
fi

# $8 - laser_model

# $9 - base_type

if [ $# -lt 10 ]; then
  MULTI="false"
  ROBOT_NAMESPACE=""
else 
  if [ "$10" = "true" ]; then
    if [ $# -lt 11 ]; then
      echo "[tiago_social_navigation/navigation.sh] If MULTI is true I need the robot_namespace"
      exit 1
    else
      MULTI="true"
      ROBOT_NAMESPACE=$11
    fi
  else 
    MULTI="false"
    ROBOT_NAMESPACE=""
    echo "[tiago_social_navigation/navigation.sh] MULTI is false so I set robot_namespace to global (/)"
  fi
fi

if [ $# -lt 12 ]; then
  MAP_TOPIC="map"
else
  MAP_TOPIC=$12
fi

if [ $# -lt 13 ]; then
  if [ ! -f "$HOME/.pal/pose.yaml" ]; then
    echo "[tiago_social_navigation/navigation.sh] '$HOME/.pal/pose.yaml' not found and will be created"
    rosrun pal_navigation_sm cp_pose_to_home.sh
  fi
  LOCALIZATION_POSE_ESTIMATE="$HOME/.pal/pose.yaml"
else
  LOCALIZATION_POSE_ESTIMATE=$13
fi

if [ $# -lt 14 ]; then
  echo "[tiago_social_navigation/navigation.sh] You have to provide a path to configuration file for the localization component, use 'gazebo_nav_base.launch'"
  exit 3
else
  LOCALIZATION_CONFIG_FILE=$14
fi

if [ $# -lt 15 ]; then
  echo "[tiago_social_navigation/navigation.sh] You have to provide a path to configuration file for the mapping component, use 'gazebo_nav_base.launch'"
  exit 4
else
  MAPPING_CONFIG_FILE=$15
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
echo "[tiago_social_navigation/navigation.sh] Invoking: 'roslaunch ${ROBOT}_social_navigation $STATE.launch \r\n\t localization:=$LOCALIZATION \r\n\t mapping:=$MAPPING \r\n\t map:=$MAP \r\n\t multiple:=$MULTI \r\n\t robot_namespace:=$ROBOT_NAMESPACE \r\n\t scan_topic:=$SCAN_TOPIC \r\n\t map_topic:=$MAP_TOPIC \r\n\t pose_estimate_file:=$LOCALIZATION_POSE_ESTIMATE \r\n\t config_file:=$CONFIGURATION_FILE'"
roslaunch ${ROBOT}_social_navigation $STATE.launch localization:=$LOCALIZATION mapping:=$MAPPING map:=$MAP multiple:=$MULTI robot_namespace:=$ROBOT_NAMESPACE scan_topic:=$SCAN_TOPIC map_topic:=$MAP_TOPIC pose_estimate_file:=$LOCALIZATION_POSE_ESTIMATE config_file:=$CONFIGURATION_FILE
