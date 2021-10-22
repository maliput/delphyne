##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cc)

##############################################################################
# Library
##############################################################################

add_library(roads_utilities ${SOURCES})

add_library(delphyne::roads_utilities ALIAS roads_utilities)

set_target_properties(
  roads_utilities
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_roads_utilities
)

target_include_directories(
  roads_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  roads_utilities
    drake::drake
    maliput::api
    maliput::common
    maliput::plugin
)

install(
  TARGETS roads_utilities
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
