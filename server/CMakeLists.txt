

include(GNUInstallDirs)


# Set the compiler flags - Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options(-Wall -Wextra -Wuninitialized -Wno-unused-variable -Wno-double-promotion -Wno-unused-parameter -Wmissing-declarations -Wno-unused-function -Wno-maybe-uninitialized)

# Enable compile optimizations for Qualcomm ARM64 (ARMv8.2-A)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -O3 -ffast-math -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ftree-vectorize")

# Find critical Packages
find_package(Eigen3 REQUIRED)

# Skip find_package if OpenCV was already configured (e.g., for cross-compilation)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 QUIET COMPONENTS core imgproc calib3d features2d highgui imgcodecs)
    if (NOT OpenCV_FOUND)
        find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc calib3d features2d highgui imgcodecs)
    endif()
endif()
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
# Try modern CMake config first
include(FetchContent)

# Fetch yaml-cpp from GitHub
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.6.3
)

# Disable yaml-cpp tests
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
# static: the drone loads ROS Foxy's yaml-cpp 0.6.2, which is ABI-incompatible
# with the 0.6.3 headers this builds against
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)
# Confirm target exists
if (TARGET yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

# Find extras


find_library(MODAL_FLOW_LIBRARY modal_flow REQUIRED HINTS /usr/lib64)
find_library(OPENCL_LIBRARY NAMES OpenCL libOpenCL.so HINTS /usr/lib64)



# Create the executable
set(SERVERNAME voxl-open-vins-server)


# tell the linker not to worry about missing symbols in libraries
set(CMAKE_C_FLAGS   "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_CXX_FLAGS}")

# List source files
set(SRC_FILES
    # VoxlCore
    VoxlCore/VoxlSpinner.cpp
    VoxlCore/VoxlVars.cpp
    VoxlCore/VoxlPublisher.cpp
    VoxlCore/VoxlHealth.cpp
    VoxlCore/VoxlConfigure.cpp
    
    # VoxlCam
    VoxlCam/CameraBase.cpp
    VoxlCam/CameraManager.cpp
    VoxlCam/CameraQueueFusion.cpp
    VoxlCam/MonoCameraMinimal.cpp
    VoxlCam/StereoCameraMinimal.cpp
    
    # VoxlIMU
    VoxlIMU/ImuMinimal.cpp
)

# Find header files
file(GLOB VOXL_CORE_HEADERS 
    "VoxlCore/*.h"
)

file(GLOB VOXL_CAM_HEADERS 
    "VoxlCam/*.h"
)

file(GLOB VOXL_IMU_HEADERS 
    "VoxlIMU/*.h"
)

# Add the target headers to the list of sources
list(APPEND SRC_FILES ${VOXL_CORE_HEADERS} ${VOXL_CAM_HEADERS} ${VOXL_IMU_HEADERS})

add_executable(${SERVERNAME} ${SRC_FILES})

# Add all the include directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})

include_directories(
    # ${CMAKE_SOURCE_DIR}/external/open_vins/  # prioritize OpenVINS submodule headers
    ./
    ./VoxlCore
    ./VoxlCam
    ./VoxlIMU
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    /usr/local/include/
    /usr/include/boost
)

# Add the target libraries
target_link_libraries(${SERVERNAME}
    ov_core_lib
    ov_msckf_lib
    ov_init_lib
    yaml-cpp
    pthread
    modal_json
    modal_pipe
    ${MODAL_FLOW_LIBRARY}
    ${CERES_LIBRARIES}
    voxl_common_config
    ${OpenCV_LIBRARIES}
    modalcv
    ${OPENCL_LIBRARY}
    ${Boost_LIBRARIES}
)


# The stuff to install 
install(
    TARGETS ${SERVERNAME}
    LIBRARY         DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME         DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
) 
