cmake_minimum_required(VERSION 3.5)
project(tiago_social_navigation)

# Based on pal_navigation_sm/CMakeLists.txt

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    tiago_2dnav_gazebo
)

## catkin specific configuration
# No packages marked as CATKIN_DEPENDS since this package does not require compilation
catkin_package()

## Install
install(
  PROGRAMS
    scripts/navigation.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ref: https://github.com/pal-robotics/tiago_robot/blob/kinetic-devel/tiago_bringup/CMakeLists.txt
foreach(dir config launch rviz)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
