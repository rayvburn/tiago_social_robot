cmake_minimum_required(VERSION 3.5)
project(tiago_social_bringup)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

## catkin specific configuration
# No packages marked as CATKIN_DEPENDS since this package does not require compilation
catkin_package()

## Install
foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
