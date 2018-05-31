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
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}>
)

target_link_libraries(
  maliput_utilities
  PUBLIC
    drake::drake
)

install(
  TARGETS maliput_utilities
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
