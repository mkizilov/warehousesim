add_executable(warehousesim_cli cli/main.cpp)
target_include_directories(warehousesim_cli PRIVATE common)
target_link_libraries(warehousesim_cli PRIVATE warehousesim::simulation)
warehousesim_configure_target(warehousesim_cli)

if(WAREHOUSESIM_BUILD_GUI)
  add_executable(warehousesim_gui gui/main.cpp)
  target_include_directories(warehousesim_gui PRIVATE common)
  target_link_libraries(warehousesim_gui PRIVATE warehousesim::rendering warehousesim::view
                                                 warehousesim::simulation)
  warehousesim_configure_target(warehousesim_gui)
endif()
