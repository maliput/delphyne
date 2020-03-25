##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cc)

##############################################################################
# Library
##############################################################################

add_library(drake_systems ${SOURCES})

set_target_properties(drake_systems PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-drake-systems)
target_include_directories(
  drake_systems
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

ament_target_dependencies(drake_systems
  "drake"
  "maliput"
  "maliput_dragway"
)

target_link_libraries(
  drake_systems
    drake::drake
    drake_gen
    maliput::api
    maliput::common
    maliput_dragway::maliput_dragway
    public_headers
)
install(
  TARGETS drake_systems
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
