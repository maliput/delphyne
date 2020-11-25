##############################################################################
# Delphyne utility
##############################################################################

add_library(utility
  compression.cc
  filesystem.cc
  package.cc
  resources/inspection.cc
  resources/meshes.cc
  resources/resources.cc
  signal_guard.cc
)

add_library(delphyne::utility ALIAS utility)

set_target_properties(utility PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_utility)

target_include_directories(
  utility
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${LIBZIP_INCLUDE_DIRS}
)

target_link_libraries(utility
  PUBLIC
    delphyne::public_headers
    ignition-common3::ignition-common3
    ${LIBZIP_LIBRARIES}
)

install(
  TARGETS utility
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
