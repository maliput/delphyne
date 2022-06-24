###############################################################################
# Install
###############################################################################

install(
  FILES
    find_lane.h
    road_builder.h
    road_network.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/roads
)
