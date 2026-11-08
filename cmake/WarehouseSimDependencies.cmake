# Every dependency is looked up with find_package first (system or package-manager
# installs) and downloaded at a pinned version only if that fails. To build offline, set
# FETCHCONTENT_SOURCE_DIR_<NAME> or FETCHCONTENT_FULLY_DISCONNECTED and provide the
# packages through CMAKE_PREFIX_PATH.
include(FetchContent)

FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.12.0
  GIT_SHALLOW TRUE
  FIND_PACKAGE_ARGS 3.11 CONFIG)
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)

if(WAREHOUSESIM_BUILD_TESTS)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    GIT_SHALLOW TRUE
    FIND_PACKAGE_ARGS NAMES GTest)
  set(INSTALL_GTEST OFF CACHE INTERNAL "")
  set(gtest_force_shared_crt ON CACHE INTERNAL "")
  FetchContent_MakeAvailable(googletest)
endif()

if(WAREHOUSESIM_BUILD_GUI)
  FetchContent_Declare(SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.2.0
    GIT_SHALLOW TRUE
    FIND_PACKAGE_ARGS 3.2 CONFIG)
  set(SDL_TEST_LIBRARY OFF CACHE INTERNAL "")
  set(SDL_TESTS OFF CACHE INTERNAL "")
  set(SDL_DISABLE_INSTALL ON CACHE INTERNAL "")
  FetchContent_MakeAvailable(SDL3)
endif()
