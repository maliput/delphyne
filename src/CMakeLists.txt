##############################################################################
# Targets
##############################################################################

### public_headers
add_library(public_headers INTERFACE)

add_library(delphyne::public_headers ALIAS public_headers)

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
)

### mi6
add_library(mi6 INTERFACE)

add_library(delphyne::mi6 ALIAS mi6)

target_include_directories(
  mi6
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mi6
  INTERFACE
    drake::drake
    delphyne::public_headers
)

##############################################################################
# Installs
##############################################################################

# Exports
install(TARGETS
  mi6 public_headers EXPORT ${PROJECT_NAME}-targets)

##############################################################################
# Directories
##############################################################################

add_subdirectory(agents)
add_subdirectory(backend)
add_subdirectory(roads)
add_subdirectory(gen)
add_subdirectory(protobuf)
add_subdirectory(systems)
add_subdirectory(translations)
add_subdirectory(utility)
add_subdirectory(visualization)
