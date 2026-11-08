warehousesim_add_module(tasks
  SOURCES dispatcher.cpp distance_estimator.cpp task.cpp task_generator.cpp task_queue.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::robots warehousesim::planning warehousesim::world)
