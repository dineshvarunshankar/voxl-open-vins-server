cmake_minimum_required(VERSION 3.22)

project(voxl-open-vins-server)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# Set the C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

message(" DEBUG BUILD ")

#memwatch
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -Wno-double-promotion -Wno-pointer-arith -DMEMWATCH -DMEMWATCH_STDIO")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMEMWATCH -DMEMWATCH_STDIO")

#SA
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -fsanitize=address -g")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a -fsanitize=address -g")

#production
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -Wno-double-promotion -Wno-pointer-arith")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")


if(EN_ION_BUF)
    add_definitions(-DEN_ION_BUF)
    message(STATUS "EN_ION_BUF enabled")
endif()
 
find_package(Boost 1.63.0 REQUIRED system filesystem)

set(CMAKE_INSTALL_LIBDIR "/usr/lib")
set(CMAKE_INSTALL_INCLUDEDIR "/usr/include")


# server and client binaries
add_subdirectory(external)
add_subdirectory(server)
add_subdirectory(clients)

