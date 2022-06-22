cmake_minimum_required(VERSION 3.3)

include(GNUInstallDirs)

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
find_package(Boost REQUIRED )

if (BUILD_FOR_X86)
    set(MODAL_PIPE_LIBRARY modal_pipe)
    set(MODAL_JSON_LIBRARY modal_json)
    set(VOXL_COMMON_CONFIG_LIBRARY voxl_common_config)
else()
    # Find all the modal AI packages
    find_library(MODAL_PIPE_LIBRARY modal_pipe REQUIRED HINTS /usr/lib64)
    find_library(MODAL_JSON_LIBRARY modal_json REQUIRED HINTS /usr/lib64)
    # find_library(MODAL_CV_LIBRARY modalcv REQUIRED HINTS /usr/lib64)
    find_library(VOXL_COMMON_CONFIG_LIBRARY voxl_common_config HINTS /usr/lib64)
    find_library(LIBRC_MATH_LIBRARY NAMES librc_math.so HINTS /usr/lib /usr/lib64 )
endif()


# Create the Server app
set(SERVERNAME voxl-open-vins-server)

# Add all the source files to the app
file(GLOB all_src_files "*.cpp" "*.c")
add_executable(${SERVERNAME} ${all_src_files})

# Add all the include directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${SERVERNAME} ${ov_core_SOURCE_DIR}/src)
include_directories(SYSTEM ${SERVERNAME} ${ov_msckf_SOURCE_DIR}/src)

include_directories(
    ./
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${voxl_common_config_INCLUDE_DIRS}
    /usr/local/include/
    /usr/include/boost
)

message(WARNING ${Boost_LIBRARIES}     ${Boost_INCLUDE_DIRS}     ${OpenCV_INCLUDE_DIRS})
# Add the target libraries
target_link_libraries(${SERVERNAME}
    pthread
    ${MODAL_JSON_LIBRARY}
    ${MODAL_PIPE_LIBRARY}
    ${LIBRC_MATH_LIBRARY}
    # ${MODAL_CV_LIBRARY}
    ${VOXL_COMMON_CONFIG_LIBRARY}
    /usr/local/lib/libov_core_lib.so
    /usr/local/lib/libov_msckf_lib.so
    /usr/lib64/libceres.a

    ${OpenCV_LIBRARIES}
    boost_system
    boost_date_time
    boost_filesystem    # ${Boost_LIBRARIES}
)

# The stuff to install 
install(
    TARGETS ${SERVERNAME}
    LIBRARY         DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME         DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
