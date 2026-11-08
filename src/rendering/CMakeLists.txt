warehousesim_add_module(rendering
  SOURCES sdl_window.cpp viewport.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::world warehousesim::simulation
  PRIVATE_DEPS SDL3::SDL3)

# Treat SDL's headers as system headers so our strict warnings never apply to them.
get_target_property(WAREHOUSESIM_SDL_INCLUDES SDL3::SDL3 INTERFACE_INCLUDE_DIRECTORIES)
if(WAREHOUSESIM_SDL_INCLUDES)
  target_include_directories(warehousesim_rendering SYSTEM PRIVATE ${WAREHOUSESIM_SDL_INCLUDES})
endif()
