cmake_minimum_required(VERSION 3.15)
project(simURDF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.1.7 REQUIRED)

coppeliasim_add_plugin(
    simURDF
    LEGACY
    SOURCES
    sourceCode/simURDF.cpp
    external/tinyxml2/tinyxml2.cpp
    sourceCode/robot.cpp
    sourceCode/link.cpp
    sourceCode/joint.cpp
    sourceCode/sensor.cpp
    sourceCode/commonFunctions.cpp
    sourceCode/rospackagehelper.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3X3Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4X4Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/mXnMatrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/7Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/mathFuncs.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/scriptFunctionData.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/scriptFunctionDataItem.cpp
)
target_include_directories(simURDF PRIVATE external)
target_compile_definitions(simURDF PRIVATE SIM_MATH_DOUBLE)
coppeliasim_add_addon("addOns/URDF importer.lua")
coppeliasim_add_addon("addOns/URDF exporter.lua")
coppeliasim_add_lua(lua/simURDF.lua)
coppeliasim_add_lua(lua/simURDF-ce.lua)
coppeliasim_add_helpfile(helpFiles/simURDF.htm)
coppeliasim_add_helpfile(helpFiles/index/simURDF.json SUBDIR index)
