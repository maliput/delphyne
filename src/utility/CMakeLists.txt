##############################################################################
# Delphyne utility
##############################################################################

add_library(${PROJECT_NAME}-utilities
  compression.cc
  filesystem.cc
  package.cc
  resources/inspection.cc
  resources/meshes.cc
  resources/resources.cc
)

target_include_directories(
  ${PROJECT_NAME}-utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${LIBZIP_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}-utilities
  PUBLIC
    public_headers
    ignition-common3::ignition-common3
    ${LIBZIP_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}-utilities EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
