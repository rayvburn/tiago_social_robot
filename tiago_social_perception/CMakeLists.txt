cmake_minimum_required(VERSION 3.5)
project(tiago_social_perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## catkin specific configuration
# No packages marked as CATKIN_DEPENDS since this package does not require compilation
catkin_package()

## Install
# ref: https://github.com/pal-robotics/tiago_robot/blob/kinetic-devel/tiago_bringup/CMakeLists.txt
foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
