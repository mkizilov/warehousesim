find_package(Threads REQUIRED)

warehousesim_add_module(core
  SOURCES clock.cpp fixed_timestep.cpp rng.cpp thread_pool.cpp
  PUBLIC_DEPS Threads::Threads)
