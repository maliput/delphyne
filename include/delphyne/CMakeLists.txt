##############################################################################
# Header Library
##############################################################################

# TODO(daniel.stonier) : Consider keeping it simple and exporting a single
# delphyne target. Caveat: might want to split concerns
# between developers and visualiser requirements.

add_library(public_headers INTERFACE) # should list the headers here?

target_include_directories(
  public_headers
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(public_headers
  INTERFACE
    drake::drake
)

# Install files
install(
  FILES
    agent_base.h
    agent_plugin_base.h
    names.h
  DESTINATION
    ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}
)

# Install the exported target
install(TARGETS public_headers EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

##############################################################################
# Subdirectories
##############################################################################

add_subdirectory(maliput)
