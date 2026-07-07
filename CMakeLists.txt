cmake_minimum_required(VERSION 3.22)

project(voxl-open-vins-server)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Set the C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE} | optimization: -O3 -ffast-math -ftree-vectorize (ARMv8.2-A)")

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

# Detect OpenCL ONCE here and define HAVE_OPENCL GLOBALLY (before any add_subdirectory) so that
# external/ (ov_core, ov_msckf), server/, and clients/ ALL compile with the SAME value. This macro
# is not cosmetic: ov_core/src/utils/sensor_data.h gates real members of ov_core::CameraData on it
# (img_frames, cl_images) plus the <CL/cl.h> include. Previously HAVE_OPENCL was set only inside
# ov_core/CMakeLists.txt via directory-scoped add_definitions(), so it never reached the sibling
# server/ subdir -- the server then saw a CameraData with no img_frames and undefined cl_* types
# (the build break), and even when it "worked" the struct layout differed across the ov_core<->server
# link boundary (an ODR/ABI mismatch). Defining it at top-level directory scope fixes both.
find_package(OpenCL QUIET)
if(OpenCL_FOUND)
    message(STATUS "OpenCL found (${OpenCL_VERSION}) - GPU path enabled globally (HAVE_OPENCL=1)")
    include_directories(${OpenCL_INCLUDE_DIRS})
    add_definitions(-DHAVE_OPENCL=1)
else()
    message(STATUS "OpenCL not found - building CPU-only globally (HAVE_OPENCL=0)")
    add_definitions(-DHAVE_OPENCL=0)
endif()

# Tolerate undefined symbols that come from the TRANSITIVE deps of shared libraries we link, e.g.
# TrackOCL -> libmodal_flow.so -> libmodal_pipe.so -> libmodal_json.so / libgbm.so. Those indirect
# libs are not on each executable's link line, but they ARE present at runtime (/usr/lib64, resolved
# via the DT_NEEDED chain + ldconfig). This is the minimal fix instead of re-enumerating the whole
# transitive closure on every target. Set before the subdirs so all binaries inherit it. (Executables
# still fail on undefined symbols in their OWN objects -- only shared-lib indirect deps are tolerated.)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--allow-shlib-undefined")

# server and client binaries
add_subdirectory(external)
add_subdirectory(server)
add_subdirectory(clients)
