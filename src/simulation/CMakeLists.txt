find_package(Threads REQUIRED)

warehousesim_add_module(simulation
  SOURCES cooperation.cpp runner.cpp scenarios.cpp serialization.cpp simulation.cpp
  PUBLIC_DEPS
    warehousesim::core warehousesim::world warehousesim::robots
    warehousesim::planning warehousesim::tasks warehousesim::traffic
    nlohmann_json::nlohmann_json Threads::Threads)
