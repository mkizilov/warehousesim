warehousesim_add_module(editor
  SOURCES editor_files.cpp map_editor.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::world
  PRIVATE_DEPS warehousesim::mapio)
