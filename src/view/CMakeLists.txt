warehousesim_add_module(view
  SOURCES camera.cpp editor_controller.cpp editor_scene.cpp fleet_controller.cpp fleet_layout.cpp
          fleet_scene.cpp fleet_session.cpp interpolation.cpp scene.cpp scene_parts.cpp view_controller.cpp
  PUBLIC_DEPS warehousesim::core warehousesim::world warehousesim::simulation warehousesim::editor)
