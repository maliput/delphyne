###############################################################################
# Install
###############################################################################

add_library(mi6 INTERFACE)

add_library(delphyne::mi6 ALIAS mi6)

target_include_directories(
  mi6
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(mi6
  INTERFACE
    drake::drake
    delphyne::public_headers
)

# Install files
install(
  FILES
    agent_base.h
    agent_base_blueprint.h
    agent_diagram_builder.h
    agent_simulation.h
    diagram_bundle.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/mi6
)

# Install the exported target
install(
  TARGETS mi6
  EXPORT ${PROJECT_NAME}-targets
)
