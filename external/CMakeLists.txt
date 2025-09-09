project(voxl-open-vins)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set the RPATH
# set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;/usr/local/lib64;/usr/local/lib;/usr/lib/aarch64-linux-gnu/")
# set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/usr/lib64;")
set(CMAKE_INSTALL_LIBDIR "/usr/lib")
set(CMAKE_INSTALL_INCLUDEDIR "/usr/include")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

add_compile_options(-Wno-unused-but-set-variable -Wno-unused-variable)

# Set the options for this vins (to not build with ROS and some other things) and then add it
set(ENABLE_CATKIN_ROS OFF CACHE BOOL "ENABLE_CATKIN_ROS")
set(ENABLE_ARUCO_TAGS OFF CACHE BOOL "ENABLE_ARUCO_TAGS")

# Dont build the evaluation code
set(BUILD_OV_EVAL OFF CACHE BOOL "BUILD_OV_EVAL")

# Add all the 3rd party libs
add_subdirectory(open_vins)
