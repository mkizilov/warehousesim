add_executable(warehousesim_cli cli/main.cpp)
target_include_directories(warehousesim_cli PRIVATE common)
target_link_libraries(warehousesim_cli PRIVATE warehousesim::simulation warehousesim::mapio)
warehousesim_configure_target(warehousesim_cli)
set(warehousesim_apps warehousesim_cli)

if(WAREHOUSESIM_BUILD_GUI)
  add_executable(warehousesim_gui gui/main.cpp)
  target_include_directories(warehousesim_gui PRIVATE common)
  target_link_libraries(warehousesim_gui PRIVATE warehousesim::rendering warehousesim::view
                                                 warehousesim::simulation warehousesim::mapio)
  warehousesim_configure_target(warehousesim_gui)

  # The editor module is already the library target warehousesim_editor, so the app's target has
  # another name; the program is still called warehousesim_editor.
  add_executable(warehousesim_editor_app editor/main.cpp)
  set_target_properties(warehousesim_editor_app PROPERTIES OUTPUT_NAME warehousesim_editor)
  target_include_directories(warehousesim_editor_app PRIVATE common)
  target_link_libraries(warehousesim_editor_app PRIVATE warehousesim::rendering warehousesim::view
                                                        warehousesim::editor warehousesim::mapio)
  warehousesim_configure_target(warehousesim_editor_app)
  list(APPEND warehousesim_apps warehousesim_gui warehousesim_editor_app)
endif()

foreach(app IN LISTS warehousesim_apps)
  target_compile_definitions(${app} PRIVATE WAREHOUSESIM_VERSION="${PROJECT_VERSION}")
endforeach()

install(TARGETS ${warehousesim_apps} RUNTIME DESTINATION bin)
