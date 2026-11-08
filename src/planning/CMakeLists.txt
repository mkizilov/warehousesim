warehousesim_add_module(planning
  SOURCES astar_planner.cpp space_time_astar.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::world)
