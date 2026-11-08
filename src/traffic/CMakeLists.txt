warehousesim_add_module(traffic
  SOURCES reservation_table.cpp traffic_controller.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::world warehousesim::robots)
