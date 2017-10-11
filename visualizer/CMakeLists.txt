include (${project_cmake_dir}/Utils.cmake)

# RenderWidget GUI plugin.
set(render_widget RenderWidget)
QT5_WRAP_CPP(${render_widget}_MOC ${render_widget}.hh)
add_library(${render_widget} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/${render_widget}.cc
  OrbitViewControl.cc
  ${${render_widget}_MOC}
)
target_link_libraries(${render_widget}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-GUI_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES})

include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/include
)

# Visualizer
add_executable(visualizer
  visualizer.cc
)

target_link_libraries(visualizer
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-GUI_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${render_widget}
)
