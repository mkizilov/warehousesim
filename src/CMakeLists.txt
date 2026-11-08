# Dependency direction (each module may only use the ones to its left):
#   core <- world <- planning
#   core <- robots <- tasks
#   {world, planning, robots, tasks} <- simulation <- rendering (SDL3)
add_subdirectory(core)
add_subdirectory(world)
add_subdirectory(robots)
add_subdirectory(planning)
add_subdirectory(tasks)
add_subdirectory(simulation)

if(WAREHOUSESIM_BUILD_GUI)
  add_subdirectory(rendering)
endif()
