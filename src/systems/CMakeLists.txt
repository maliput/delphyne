##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cc)

##############################################################################
# Library
##############################################################################

add_library(drake_agents ${SOURCES})
set_target_properties(drake_agents PROPERTIES OUTPUT_NAME drake-agents)
target_include_directories(
  drake_agents
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}>
)
target_link_libraries(
  drake_agents
  PUBLIC
    drake::drake
)
