###############################################################################
# Install
###############################################################################

install(
  FILES
    find_lane.h
    road_builder.h
    road_network_wrapper.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/roads
)
