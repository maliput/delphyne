##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cc)

##############################################################################
# Library
##############################################################################

add_library(roads_utilities ${SOURCES})

set_target_properties(
  roads_utilities
  PROPERTIES
    OUTPUT_NAME
      ${PROJECT_NAME}-roads-utilities
)

target_include_directories(
  roads_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(roads_utilities
  "drake"
  "maliput"
  "multilane"
  "dragway"
)

target_link_libraries(
  roads_utilities
    drake::drake
    dragway::dragway
    maliput::api
    multilane::multilane
    malidrive::utility
    malidrive::malidrive
    malidrive::opendrive
)

install(
  TARGETS roads_utilities EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
