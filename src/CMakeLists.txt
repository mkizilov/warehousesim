# Dependency direction (each module may only use the ones above it):
#   core <- world <- planning
#   core <- robots <- tasks
#   {world, planning, robots, tasks} <- simulation <- view (camera, scene; no SDL)
#   view <- rendering (SDL3 window; GUI only)
add_subdirectory(core)
add_subdirectory(world)
add_subdirectory(robots)
add_subdirectory(planning)
add_subdirectory(tasks)
add_subdirectory(simulation)
add_subdirectory(view)

if(WAREHOUSESIM_BUILD_GUI)
  add_subdirectory(rendering)
endif()
