warehousesim_add_module(tasks
  SOURCES assigner.cpp task.cpp task_generator.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::robots)
