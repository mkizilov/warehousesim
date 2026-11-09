find_package(Threads REQUIRED)

warehousesim_add_module(simulation
  SOURCES audit.cpp benchmark.cpp commands.cpp cooperation.cpp metrics_collector.cpp recording.cpp
          recording_file.cpp replay.cpp runner.cpp scenarios.cpp serialization.cpp simulation.cpp
          state_checksum.cpp
  PUBLIC_DEPS
    warehousesim::core warehousesim::world warehousesim::robots
    warehousesim::planning warehousesim::tasks warehousesim::traffic warehousesim::metrics
    nlohmann_json::nlohmann_json Threads::Threads)
