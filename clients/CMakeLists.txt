cmake_minimum_required(VERSION 3.3)



set(TARGET voxl-inspect-vins)
add_executable(${TARGET} voxl-inspect-vins.cpp)
include_directories(../common)
list(APPEND ALL_TARGETS ${TARGET})

find_library(MODAL_PIPE_LIBRARY modal_pipe REQUIRED HINTS /usr/lib64)
find_library(MODAL_JSON_LIBRARY modal_json REQUIRED HINTS /usr/lib64)


target_link_libraries(${TARGET} ${MODAL_JSON_LIBRARY} ${MODAL_PIPE_LIBRARY})

install(
	TARGETS ${ALL_TARGETS}
	LIBRARY			DESTINATION /usr/lib
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)

