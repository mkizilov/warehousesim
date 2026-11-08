cmake_minimum_required(VERSION 3.24)

project(WarehouseSim
  VERSION 0.1.0
  DESCRIPTION "Autonomous warehouse robot simulation"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

option(WAREHOUSESIM_BUILD_TESTS "Build the GoogleTest unit tests" ON)
option(WAREHOUSESIM_BUILD_GUI "Build the SDL3 graphical executable" ON)
option(WAREHOUSESIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(WAREHOUSESIM_ENABLE_CLANG_TIDY "Run clang-tidy while compiling project targets" OFF)
set(WAREHOUSESIM_SANITIZE "" CACHE STRING
  "Comma-separated sanitizers for all targets, e.g. address,undefined or thread")

# Set before the dependencies are added so they are instrumented consistently.
if(WAREHOUSESIM_SANITIZE)
  add_compile_options(-fsanitize=${WAREHOUSESIM_SANITIZE} -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${WAREHOUSESIM_SANITIZE})
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(WarehouseSimTargets)
include(WarehouseSimDependencies)

add_subdirectory(src)
add_subdirectory(apps)

if(WAREHOUSESIM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
