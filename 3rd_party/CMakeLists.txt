cmake_minimum_required(VERSION 3.3)

add_compile_options(-Wno-unused-but-set-variable -Wno-unused-variable)

# Set the options for this vins (to not build with ROS and some other things) and then add it
set(ENABLE_CATKIN_ROS OFF CACHE BOOL "ENABLE_CATKIN_ROS")
set(ENABLE_ARUCO_TAGS OFF CACHE BOOL "ENABLE_ARUCO_TAGS")

# Dont build the evaluation code
set(BUILD_OV_EVAL OFF CACHE BOOL "BUILD_OV_EVAL")

# Add all the 3rd party libs
add_subdirectory(open_vins)
