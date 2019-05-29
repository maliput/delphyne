##############################################################################
# Helper library
##############################################################################

add_library(examples
  kinematic_car_model.cc
  malidrive_automotive_simulator.cc
  opendriver_controller.cc
  pure_pursuit.cc
  railcar_logger_system.cc
)
set_target_properties(examples PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-examples)

target_include_directories(
  examples
  PUBLIC  # so the bindings can find these headers
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

ament_target_dependencies(examples
  "maliput"
  "malidrive"
)

target_link_libraries(
  examples
    drake_gen
    drake_systems
    drake::drake
    maliput::api
    malidrive::utility
    malidrive::malidrive
    malidrive::opendrive
    malidrive::backend
)

install(TARGETS examples
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)


##############################################################################
# Examples
##############################################################################

add_executable(kinematic_car_example kinematic_car_example.cc)

ament_target_dependencies(kinematic_car_example
  "malidrive")

target_link_libraries(
  kinematic_car_example
    drake::drake
    malidrive::opendrive
    malidrive::utility
    examples
)
install(
  TARGETS
     kinematic_car_example
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/examples/
)

add_executable(opendriver_controller_example opendriver_controller_example.cc)

ament_target_dependencies(opendriver_controller_example
  "malidrive")

target_link_libraries(
  opendriver_controller_example
    drake::drake
    malidrive::opendrive
    malidrive::utility
    examples
)
install(
  TARGETS
     opendriver_controller_example
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/examples/
)

add_executable(malidrive_demo malidrive_demo.cc)

ament_target_dependencies(malidrive_demo
  "maliput"
  "malidrive")

target_link_libraries(
  malidrive_demo
    drake::drake
    maliput::api
    malidrive::malidrive
    malidrive::opendrive
    malidrive::utility
    drake_gen
    examples
)
install(
  TARGETS
     malidrive_demo
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/examples/
)
