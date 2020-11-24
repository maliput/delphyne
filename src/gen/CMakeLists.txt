##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cc)

##############################################################################
# Library
##############################################################################

add_library(drake_gen ${SOURCES})

add_library(delphyne::drake_gen ALIAS drake_gen)

set_target_properties(drake_gen
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_drake_gen
    LINKER_LANGUAGE CXX
)

target_include_directories(
  drake_gen
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

ament_target_dependencies(drake_gen
  PUBLIC
    maliput
)

target_link_libraries(
  drake_gen
  PUBLIC
    drake::drake
    delphyne::public_headers
    maliput::common
)

install(
  TARGETS drake_gen
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
