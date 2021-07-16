##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cc)

##############################################################################
# Library
##############################################################################

add_library(drake_systems ${SOURCES})

add_library(delphyne::drake_systems ALIAS drake_systems)

set_target_properties(drake_systems
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_drake_systems
)

target_include_directories(
  drake_systems
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

target_link_libraries(
  drake_systems
    drake::drake
    delphyne::drake_gen
    delphyne::public_headers
    maliput::api
    maliput::common
    maliput_dragway::maliput_dragway
)

install(
  TARGETS drake_systems
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
