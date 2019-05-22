##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cc)

##############################################################################
# Library
##############################################################################

add_library(maliput_utilities ${SOURCES})

set_target_properties(
  maliput_utilities
  PROPERTIES
    OUTPUT_NAME
      ${PROJECT_NAME}-maliput-utilities
)

target_include_directories(
  maliput_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(maliput_utilities
  "drake"
  "maliput"
  "multilane"
  "dragway"
)

target_link_libraries(
  maliput_utilities
    drake::drake
    dragway::dragway
    maliput::api
    multilane::multilane
    malidrive::utility
    malidrive::malidrive
    malidrive::opendrive
)

install(
  TARGETS maliput_utilities EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
