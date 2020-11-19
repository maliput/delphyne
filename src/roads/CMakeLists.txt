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
  "maliput_dragway"
  "maliput_multilane"
)

target_link_libraries(
  roads_utilities
    drake::drake
    malidrive::builder
    malidrive::loader
    maliput::api
    maliput_dragway::maliput_dragway
    maliput_multilane::maliput_multilane
    maliput_malidrive::builder
    maliput_malidrive::loader
    maliput_malidrive::utility
)

install(
  TARGETS roads_utilities EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
