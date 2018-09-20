cmake_minimum_required(VERSION 3.2)
project(SGP30_Example
        VERSION 1.0.0
        LANGUAGES C)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include(CMakeListsPrivate.cmake)

add_custom_target(
        AirFEEL_BUILD ALL
        COMMAND make -j9 all
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        AirFEEL_UPLOAD ALL
        COMMAND make -j9 flash
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        AirFEEL_CLEAN ALL
        COMMAND make clean
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        AirFEEL_TEST ALL
        COMMAND make list-components
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        AirFEEL_PROGRAM ALL
        COMMAND make flash
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(${PROJECT_NAME} ${SRC_LIST})
