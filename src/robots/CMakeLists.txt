warehousesim_add_module(robots
  SOURCES battery.cpp forecast.cpp motion.cpp robot.cpp robot_spec.cpp robot_state.cpp
  PUBLIC_DEPS warehousesim::core)
