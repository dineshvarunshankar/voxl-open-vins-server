cmake_minimum_required(VERSION 3.3)

include(GNUInstallDirs)

# Find critical Packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()
# find_package(Boost REQUIRED )
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)


# tell the linker not to worry about missing symbols in libraries
set(CMAKE_C_FLAGS   "-Wl,--unresolved-symbols=ignore-in-shared-libs -Wno-unused-result ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs -Wno-unused-result ${CMAKE_CXX_FLAGS}")


if (BUILD_FOR_X86)
    set(MODAL_PIPE_LIBRARY modal_pipe)
    set(MODAL_JSON_LIBRARY modal_json)
    set(VOXL_COMMON_CONFIG_LIBRARY voxl_common_config)
else()
    # Find all the modal AI packages
    find_library(MODAL_PIPE_LIBRARY modal_pipe REQUIRED HINTS /usr/lib64)
    find_library(MODAL_JSON_LIBRARY modal_json REQUIRED HINTS /usr/lib64)
    find_library(VOXL_COMMON_CONFIG_LIBRARY voxl_common_config HINTS /usr/lib64)
    find_library(LIBRC_MATH_LIBRARY NAMES librc_math.so HINTS /usr/lib /usr/lib64 )
endif()

set(TARGET voxl-inspect-vins)
add_executable(${TARGET} voxl-inspect-vins.cpp)
list(APPEND ALL_TARGETS ${TARGET})

find_library(MODAL_PIPE_LIBRARY modal_pipe REQUIRED HINTS /usr/lib64)
find_library(MODAL_JSON_LIBRARY modal_json REQUIRED HINTS /usr/lib64)

target_link_libraries(${TARGET} ${MODAL_JSON_LIBRARY} ${MODAL_PIPE_LIBRARY} )


set(TARGET voxl-evaluate-vins)
add_executable(${TARGET} voxl-evaluate-vins.cpp)
list(APPEND ALL_TARGETS ${TARGET})

target_link_libraries(${TARGET} ${MODAL_JSON_LIBRARY} ${MODAL_PIPE_LIBRARY} )


install(
	TARGETS ${ALL_TARGETS}
	LIBRARY			DESTINATION /usr/lib
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)

