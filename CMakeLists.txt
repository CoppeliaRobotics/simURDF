cmake_minimum_required(VERSION 3.15)
project(simExtURDF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)

coppeliasim_add_plugin(
    simExtURDF
    LEGACY
    SOURCES
    simExtURDF.cpp
    tinyxml2/tinyxml2.cpp
    robot.cpp
    link.cpp
    joint.cpp
    sensor.cpp
    commonFunctions.cpp
    rospackagehelper.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/3Vector.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/3X3Matrix.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/4Vector.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/4X4Matrix.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/MMatrix.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/7Vector.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/MyMath.cpp
    ${COPPELIASIM_COMMON_DIR}/scriptFunctionData.cpp
    ${COPPELIASIM_COMMON_DIR}/scriptFunctionDataItem.cpp
)
target_include_directories(simExtURDF PRIVATE ${COPPELIASIM_ROOT_DIR}/programming/simMath)
coppeliasim_add_addon("simAddOnURDF importer.lua")
coppeliasim_add_addon("simAddOnURDF exporter.lua")
coppeliasim_add_lua(simURDF.lua)
coppeliasim_add_helpfile(simURDF.htm)
coppeliasim_add_helpfile(simURDF.json SUBDIR index)
