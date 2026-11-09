cmake_minimum_required(VERSION 3.24)

project(WarehouseSim
  VERSION 1.0.0
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
option(WAREHOUSESIM_BUILD_BENCHMARKS "Build the planning benchmark" ON)
option(WAREHOUSESIM_BUILD_GUI "Build the SDL3 graphical executable" ON)
option(WAREHOUSESIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(WAREHOUSESIM_ENABLE_CLANG_TIDY "Run clang-tidy while compiling project targets" OFF)
set(WAREHOUSESIM_SANITIZE "" CACHE STRING
  "Comma-separated sanitizers for all targets, e.g. address,undefined or thread")

# Set before the dependencies are added so they are instrumented consistently.
if(WAREHOUSESIM_SANITIZE)
  add_compile_options(-fsanitize=${WAREHOUSESIM_SANITIZE} -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${WAREHOUSESIM_SANITIZE})
  # A sanitizer that reports and carries on lets a test pass with the report scrolled away; any
  # finding must end the program with a failure instead.
  add_compile_options(-fno-sanitize-recover=all)
endif()

# Static libraries that are needed both directly (for their headers) and through another library
# are named twice on the link line; Apple's linker says so for each of them. It is harmless.
if(APPLE)
  include(CheckLinkerFlag)
  check_linker_flag(CXX "LINKER:-no_warn_duplicate_libraries" WAREHOUSESIM_LINKER_QUIET_DUPLICATES)
  if(WAREHOUSESIM_LINKER_QUIET_DUPLICATES)
    add_link_options("LINKER:-no_warn_duplicate_libraries")
  endif()
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(WarehouseSimTargets)
include(WarehouseSimDependencies)

if(WAREHOUSESIM_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(testing)

if(WAREHOUSESIM_BUILD_BENCHMARKS)
  add_subdirectory(bench)
endif()

if(WAREHOUSESIM_BUILD_TESTS)
  add_subdirectory(tests)
  add_subdirectory(examples)
endif()

# ---- install and package -----------------------------------------------------------------------
# cmake --install build/release --prefix DIR, or cpack (the `package` build preset): the programs,
# the example maps and the documentation. The libraries are internal and are not installed.
install(DIRECTORY examples/ DESTINATION share/warehousesim/examples FILES_MATCHING PATTERN "*.json")
install(DIRECTORY docs/ DESTINATION share/doc/warehousesim FILES_MATCHING PATTERN "*.md")
install(FILES LICENSE README.md CHANGELOG.md DESTINATION share/doc/warehousesim)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(CPACK_PACKAGE_NAME warehousesim)
  set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
  set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
  set(CPACK_PACKAGE_FILE_NAME
    "warehousesim-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
  set(CPACK_GENERATOR TGZ)
  if(WIN32)
    set(CPACK_GENERATOR ZIP)
  endif()
  include(CPack)
endif()
