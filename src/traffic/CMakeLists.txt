warehousesim_add_module(traffic
  SOURCES cooperative_router.cpp progress_monitor.cpp reservation_table.cpp traffic_controller.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::world warehousesim::robots warehousesim::planning)
