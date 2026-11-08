# JSON is an implementation detail of the map file: nothing in the public header names it.
warehousesim_add_module(mapio
  SOURCES map_file.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::world
  PRIVATE_DEPS nlohmann_json::nlohmann_json)
