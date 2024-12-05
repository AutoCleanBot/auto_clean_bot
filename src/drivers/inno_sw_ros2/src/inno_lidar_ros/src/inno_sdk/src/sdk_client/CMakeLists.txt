project(innolidarsdkclient)
cmake_minimum_required(VERSION 3.0)

include(${PROJECT_SOURCE_DIR}/../../build/inno_lidar_base.cmake)

if(PCAP_INPUT)
    add_definitions(-DPCAP_INPUT=1)
endif()

aux_source_directory(. DIR_SRCS)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND MAKE_SHARED)
    add_library(${PROJECT_NAME} SHARED ${DIR_SRCS})
    set_target_properties(${PROJECT_NAME} PROPERTIES VERSION
    ${INNO_SDK_VERSION_MAJOR}.${INNO_SDK_VERSION_MINOR}.${INNO_SDK_VERSION_PATCH} SOVERSION ${INNO_SDK_VERSION_MAJOR})
    target_link_libraries(${PROJECT_NAME} PRIVATE innolidarsdkcommon innolidarutils pthread)
    if(PCAP_INPUT)
        target_link_libraries(${PROJECT_NAME} PRIVATE pcap)
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows" AND MAKE_SHARED)
    add_library(${PROJECT_NAME} SHARED ${DIR_SRCS})
    target_link_libraries(${PROJECT_NAME} PRIVATE innolidarsdkcommon innolidarutils ws2_32 wsock32)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin" AND MAKE_SHARED)
    add_library(${PROJECT_NAME} SHARED ${DIR_SRCS})
    target_link_libraries(${PROJECT_NAME} PRIVATE innolidarsdkcommon innolidarutils pthread)
else()
    add_library(${PROJECT_NAME} ${DIR_SRCS})
endif()


set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../lib)

########################
# add_custom_command(TARGET ${PROJECT_NAME}
#     POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_SOURCE_DIR}/../../lib
#     COMMENT "Copying library to current working directory")

# INSTALL(TARGETS ${PROJECT_NAME}
#     LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/../../lib
#     RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/../../lib
#     ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/../../lib)
