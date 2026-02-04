cmake_minimum_required(VERSION 3.22)

project(voxl-open-vins-server)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

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

# Production flags for Qualcomm ARM64 embedded (ARMv8.2-A: QCS6490/QRB5165)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -O3 -ffast-math -fomit-frame-pointer -funroll-loops -ftree-vectorize -Wno-double-promotion -Wno-pointer-arith")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8.2-a -O3 -ffast-math -fomit-frame-pointer -funroll-loops -ftree-vectorize")


if(EN_ION_BUF)
    add_definitions(-DEN_ION_BUF)
    message(STATUS "EN_ION_BUF enabled")
endif()

find_package(Boost 1.63.0 REQUIRED system filesystem)

set(CMAKE_INSTALL_LIBDIR "/usr/lib")
set(CMAKE_INSTALL_INCLUDEDIR "/usr/include")

# Disable TrackKLT by default (requires opencv video module)
set(DISABLE_TRACK_KLT ON CACHE BOOL "Disable TrackKLT to avoid opencv video module dependency")
if(DISABLE_TRACK_KLT)
    add_definitions(-DDISABLE_TRACK_KLT=1)
    message(STATUS "TrackKLT DISABLED globally")
endif()

# Platform-specific OpenCV setup (PLATFORM is set by toolchain file)
if(PLATFORM STREQUAL "QCS6490")
    # QCS6490: Manual OpenCV setup (minimal headers, no cmake config)
    message(STATUS "QCS6490 build: Using manual OpenCV component linking")

    set(QCS_SYSROOT "/opt/sysroots/qcs6490")
    set(OpenCV_FOUND TRUE CACHE BOOL "" FORCE)
    set(OpenCV_VERSION "4.0.0" CACHE STRING "" FORCE)
    set(OpenCV_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/opencv_compat;${QCS_SYSROOT}/usr/include/opencv4" CACHE PATH "" FORCE)

    # Include compat dir FIRST to override opencv.hpp with our minimal version
    include_directories(SYSTEM "${CMAKE_SOURCE_DIR}/opencv_compat")
    include_directories(SYSTEM "${QCS_SYSROOT}/usr/include/opencv4")

    # Find OpenCV libraries in sysroot
    set(QCS_LIB_PATH "${QCS_SYSROOT}/usr/lib/aarch64-linux-gnu")
    set(OpenCV_LIBS_LIST
        opencv_core
        opencv_imgproc
        opencv_calib3d
        opencv_features2d
        opencv_highgui
        opencv_imgcodecs
        opencv_flann
    )
    set(OpenCV_LIBRARIES "")
    foreach(lib ${OpenCV_LIBS_LIST})
        find_library(${lib}_LIB NAMES ${lib} HINTS ${QCS_LIB_PATH} NO_DEFAULT_PATH)
        if(${lib}_LIB)
            list(APPEND OpenCV_LIBRARIES ${${lib}_LIB})
        endif()
    endforeach()
    set(OpenCV_LIBRARIES ${OpenCV_LIBRARIES} CACHE STRING "" FORCE)

    message(STATUS "QCS6490 OpenCV includes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "QCS6490 OpenCV libraries: ${OpenCV_LIBRARIES}")

elseif(PLATFORM STREQUAL "QRB5165")
    # QRB5165: Use standard find_package (full OpenCV available)
    message(STATUS "QRB5165 build: Using standard OpenCV find_package")
    # OpenCV_LIBS and OpenCV_INCLUDE_DIRS are set by toolchain

else()
    # Native or unknown platform
    message(STATUS "Native/unknown platform: Using standard OpenCV find_package")
endif()

# server and client binaries
add_subdirectory(external)
add_subdirectory(server)
add_subdirectory(clients)
