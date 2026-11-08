warehousesim_add_module(view
  SOURCES camera.cpp interpolation.cpp scene.cpp view_controller.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::world warehousesim::simulation)
