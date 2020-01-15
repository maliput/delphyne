##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cc)

##############################################################################
# Library
##############################################################################

add_library(drake_gen ${SOURCES})

set_target_properties(drake_gen PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-drake-gen LINKER_LANGUAGE CXX)
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
    maliput::common
    public_headers
    drake::drake
)
install(
  TARGETS drake_gen
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
