# Dependency direction (each module may only use the ones above it):
#   core <- world <- planning
#   core <- robots <- tasks
#   {world, robots} <- traffic (space-time reservations)
#   {world, planning, robots, tasks, traffic} <- simulation
#   world <- mapio (versioned JSON map file) <- editor (editing model)
#   {simulation, editor} <- view (camera, scenes, controllers; no SDL)
#   view <- rendering (SDL3 window; GUI only)
add_subdirectory(core)
add_subdirectory(world)
add_subdirectory(mapio)
add_subdirectory(editor)
add_subdirectory(robots)
add_subdirectory(planning)
add_subdirectory(tasks)
add_subdirectory(traffic)
add_subdirectory(simulation)
add_subdirectory(view)

if(WAREHOUSESIM_BUILD_GUI)
  add_subdirectory(rendering)
endif()
