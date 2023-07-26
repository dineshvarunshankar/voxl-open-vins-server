cmake_minimum_required(VERSION 3.3)

project(voxl-open-vins-server)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

message(" DEBUG BUILD ")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -Wno-double-promotion -Wno-pointer-arith -g")
 
find_package(Boost 1.63.0 REQUIRED system filesystem)

set(CMAKE_INSTALL_LIBDIR "/usr/lib")
set(CMAKE_INSTALL_INCLUDEDIR "/usr/include")


# server and client binaries
add_subdirectory(server)
add_subdirectory(clients)

