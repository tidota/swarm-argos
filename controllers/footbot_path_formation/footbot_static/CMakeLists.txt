add_library(footbot_static MODULE footbot_static.h footbot_static.cpp)
target_link_libraries(footbot_static
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

