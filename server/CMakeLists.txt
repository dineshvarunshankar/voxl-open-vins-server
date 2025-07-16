cmake_minimum_required(VERSION 3.22)

include(GNUInstallDirs)

option(BUILD_QRB5165 "Build the qrb5165 binary" OFF)


message(STATUS "qrb build: " ${BUILD_QRB5165})

# Set the compiler flags - Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wuninitialized -Wno-unused-variable -Wno-double-promotion -Wno-unused-parameter -Wmissing-declarations -Wno-unused-function -Wno-maybe-uninitialized)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Find critical Packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
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
set(YAML_BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE) #cross 4.0 quits the chat if building static
FetchContent_MakeAvailable(yaml-cpp)
# Confirm target exists
if (TARGET yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

install(FILES
    ${yaml-cpp_BINARY_DIR}/libyaml-cpp.so
    ${yaml-cpp_BINARY_DIR}/libyaml-cpp.so.0.6
    ${yaml-cpp_BINARY_DIR}/libyaml-cpp.so.0.6.3
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
# Find extras
find_library(MODAL_PIPE_LIBRARY modal_pipe REQUIRED HINTS /usr/lib64)
find_library(MODAL_JSON_LIBRARY modal_json REQUIRED HINTS /usr/lib64)
find_library(MODAL_FLOW_LIBRARY modal_flow REQUIRED HINTS /usr/lib64)
find_library(LIB_CERES NAMES libceres.so.1 REQUIRED HINTS /usr/lib /usr/lib64)
find_library(OPENCL_LIBRARY NAMES OpenCL libOpenCL.so HINTS /usr/lib64)
find_library(MODALCV    modalcv    HINTS  /usr/lib64)
find_library(VOXL_COMMON_CONFIG voxl_common_config HINTS  /usr/lib64)



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
# have to include this, otherwise ceres will moan about not finding glog/logging.h header
# its a minimal glog that is included with ceres, not the actual glog
include_directories(/usr/include/ceres/internal/miniglog/)

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
    ${MODAL_JSON_LIBRARY}
    ${MODAL_PIPE_LIBRARY}
    ${MODAL_FLOW_LIBRARY}
    ${LIB_CERES}
    ${VOXL_COMMON_CONFIG}
    ${OpenCV_LIBRARIES}
    ${MODALCV}
    ${OPENCL_LIBRARY}
    boost_system boost_date_time boost_filesystem
)


# The stuff to install 
install(
    TARGETS ${SERVERNAME}
    LIBRARY         DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME         DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
) 