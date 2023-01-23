cmake_minimum_required(VERSION 3.5)
project(tiago_social_navigation)

# Based on pal_navigation_sm/CMakeLists.txt

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    pal_navigation_cfg_tiago
    robot_pose
    laser_filters
    social_navigation_layers
)

## catkin specific configuration
# No packages marked as CATKIN_DEPENDS since this package does not require compilation
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

## Install
install(
  PROGRAMS
    scripts/navigation.sh
    scripts/lidar_shrunker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ref: https://github.com/pal-robotics/tiago_robot/blob/kinetic-devel/tiago_bringup/CMakeLists.txt
foreach(dir config launch rviz)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
