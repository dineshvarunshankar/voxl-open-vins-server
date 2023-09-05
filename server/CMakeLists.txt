cmake_minimum_required(VERSION 3.3)

include(GNUInstallDirs)

option(BUILD_QRB5165 "Build the qrb5165 binary" OFF)

if(BUILD_QRB5165)
    add_definitions(-DBUILD_QRB5165)
endif()

message(STATUS "qrb build: " ${BUILD_QRB5165})

# Set the compiler flags
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

# Find extras
find_library(MODAL_PIPE_LIBRARY modal_pipe REQUIRED HINTS /usr/lib64)
find_library(MODAL_JSON_LIBRARY modal_json REQUIRED HINTS /usr/lib64)
find_library(LIBOV_CORE NAMES libov_core_lib.so REQUIRED HINTS /usr/lib /usr/lib64 )
find_library(LIBOV_MSCKF NAMES libov_msckf_lib.so REQUIRED HINTS /usr/lib /usr/lib64 )
find_library(LIB_CERES NAMES libceres.so.1 REQUIRED HINTS /usr/lib /usr/lib64 )
## just a 64-bit lib, lives in lib64 since APQ8096 doesn't like to link to 64-bit
## libs in /usr/lib/
find_library(MODALCV    modalcv    HINTS  /usr/lib64)

find_library(VOXL_COMMON_CONFIG voxl_common_config HINTS  /usr/lib64)


# Create the Server app
set(SERVERNAME voxl-open-vins-server)

# tell the linker not to worry about missing symbols in libraries
set(CMAKE_C_FLAGS   "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_CXX_FLAGS}")


# Add all the source files to the app
file(GLOB all_src_files "*.cpp" "*.c")
add_executable(${SERVERNAME} ${all_src_files})

# Add all the include directories
include_directories(${EIGEN3_INCLUDE_DIR})
# have to include this, otherwise ceres will moan about not finding glog/logging.h header
# its a minimal glog that is included with ceres, not the actual glog
include_directories(/usr/include/ceres/internal/miniglog/)

include_directories(SYSTEM ${SERVERNAME} ${ov_core_SOURCE_DIR}/src)
include_directories(SYSTEM ${SERVERNAME} ${ov_msckf_SOURCE_DIR}/src)

include_directories(
    ./
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    /usr/local/include/
    /usr/include/boost
)

# Add the target libraries
target_link_libraries(${SERVERNAME}
    pthread
    ${MODAL_JSON_LIBRARY}
    ${MODAL_PIPE_LIBRARY}
    ${LIBOV_CORE}
    ${LIBOV_MSCKF}
    ${LIB_CERES}
    ${VOXL_COMMON_CONFIG}
    ${OpenCV_LIBRARIES}
    ${MODALCV}
)

if(BUILD_QRB5165)
    target_link_libraries(${SERVERNAME} boost_system boost_date_time boost_filesystem)
else()
    target_link_libraries(${SERVERNAME} ${Boost_LIBRARIES})
endif()


# The stuff to install 
install(
    TARGETS ${SERVERNAME}
    LIBRARY         DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME         DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
