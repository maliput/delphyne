###############################################################################
# Install
###############################################################################

add_library(mi6 INTERFACE)

target_include_directories(
  mi6
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(mi6
  INTERFACE
    public_headers
    drake::drake
)

# Install files
install(
  FILES
    agent_base.h
    agent_diagram.h
    agent_diagram_builder.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/mi6
)

# Install the exported target
install(TARGETS mi6 EXPORT ${PROJECT_NAME}-targets)
