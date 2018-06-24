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
target_link_libraries(
  drake_systems
  PUBLIC
    drake::drake
)
install(
  TARGETS drake_systems
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
