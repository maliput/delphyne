##############################################################################
# Configured
##############################################################################

configure_file(version.h.in
  ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/version.h
)

##############################################################################
# Targets
##############################################################################

# Can this be handled so that the interface library is exactly the headers
# we designate (in the install)?
add_library(public_headers INTERFACE)

target_include_directories(
  public_headers
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(public_headers
  INTERFACE
    drake::drake
    maliput::common
)

##############################################################################
# Installs
##############################################################################

# Files
install(
  FILES
    macros.h
    types.h
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/version.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# Exports
install(TARGETS public_headers EXPORT ${PROJECT_NAME}-targets)

##############################################################################
# Subdirectories
##############################################################################

add_subdirectory(roads)
add_subdirectory(mi6)
add_subdirectory(utility)
