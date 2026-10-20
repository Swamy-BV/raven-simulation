# SPDX-License-Identifier: AGPL-3.0-only
cmake_minimum_required(VERSION 3.20)
project(
  raven_simulation
  VERSION 1.0.0
  LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ProjectAirSimCppClient 1.0.1 EXACT REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
set(PX4_SOURCE_DIR
    ""
    CACHE PATH "Firmware-matched PX4 checkout with native SITL built")
set(XRCE_DIR "${PX4_SOURCE_DIR}/build/px4_sitl_default/src/modules/uxrce_dds_client")
if(NOT EXISTS "${XRCE_DIR}/lib/libmicroxrcedds_client.a")
  message(FATAL_ERROR "Build firmware-matched px4_sitl_default and set PX4_SOURCE_DIR")
endif()
add_executable(airsim-bridge src/main.cpp src/client.cpp src/scenario.cpp src/video.cpp)
target_compile_options(airsim-bridge PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(airsim-bridge SYSTEM PRIVATE "${XRCE_DIR}/include")
target_link_libraries(airsim-bridge PRIVATE ProjectAirSim::Client PkgConfig::GSTREAMER
                                            "${XRCE_DIR}/lib/libmicroxrcedds_client.a" "${XRCE_DIR}/lib/libmicrocdr.a")
include(CTest)
if(BUILD_TESTING)
  add_test(NAME scene-validation
           COMMAND airsim-bridge --validate "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/street-loop.jsonc"
                   "${CMAKE_CURRENT_BINARY_DIR}/validated-scene")
endif()
install(TARGETS airsim-bridge RUNTIME DESTINATION bin)
