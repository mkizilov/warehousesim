# Aggregation of simulated performance figures. It sees observations as plain data and depends on
# nothing but core, so it is tested without a simulation and can never change one.
warehousesim_add_module(metrics
  SOURCES accumulator.cpp export.cpp fleet_metrics.cpp observation.cpp
  PUBLIC_DEPS warehousesim::core)
