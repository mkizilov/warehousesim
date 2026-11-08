find_program(WAREHOUSESIM_CLANG_TIDY NAMES clang-tidy)

if(WAREHOUSESIM_ENABLE_CLANG_TIDY AND NOT WAREHOUSESIM_CLANG_TIDY)
  message(FATAL_ERROR "WAREHOUSESIM_ENABLE_CLANG_TIDY is ON but clang-tidy was not found")
endif()

# Applies the project's compiler warnings and (optionally) clang-tidy to a target.
function(warehousesim_configure_target target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
    if(WAREHOUSESIM_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    # Motion is floating-point maths that must come out the same on every platform. Fused
    # multiply-add (contracted by default on some compilers and CPUs) rounds differently, so
    # it is turned off: every result is then plain IEEE arithmetic.
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -ffp-contract=off)
    if(WAREHOUSESIM_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
  if(WAREHOUSESIM_ENABLE_CLANG_TIDY)
    set_target_properties(${target} PROPERTIES
      CXX_CLANG_TIDY "${WAREHOUSESIM_CLANG_TIDY};--warnings-as-errors=*")
  endif()
endfunction()

# Declares one module as a static library `warehousesim_<name>` (alias
# `warehousesim::<name>`). Public headers live in include/warehousesim/<name>/. A module can
# only include headers of modules it links, so the dependency direction is enforced by
# the build rather than by convention.
function(warehousesim_add_module name)
  cmake_parse_arguments(ARG "" "" "SOURCES;PUBLIC_DEPS;PRIVATE_DEPS" ${ARGN})
  add_library(warehousesim_${name} STATIC ${ARG_SOURCES})
  add_library(warehousesim::${name} ALIAS warehousesim_${name})
  target_include_directories(warehousesim_${name} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_libraries(warehousesim_${name}
    PUBLIC ${ARG_PUBLIC_DEPS}
    PRIVATE ${ARG_PRIVATE_DEPS})
  warehousesim_configure_target(warehousesim_${name})
endfunction()
