cmake_minimum_required(VERSION 3.3)

project(voxl-open-vins-server)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set the RPATH
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;/usr/local/lib64;/usr/local/lib;/usr/lib/aarch64-linux-gnu/")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -Wno-double-promotion")


# option(BUILD_FOR_X86 "Enable or disable building of for desktop computers" OFF)
# if (BUILD_FOR_X86)
# 	add_subdirectory(modalai_libs)
# endif()
 
find_package(Boost 1.63.0 REQUIRED system filesystem)
# The 3rd Party Stuff
# add_subdirectory(3rd_party)

# server and client binaries
add_subdirectory(server)
add_subdirectory(clients)

