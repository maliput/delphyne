###############################################################################
# Install
###############################################################################

install(
  FILES
    find_lane.h
    road_builder.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/maliput
)
