###############################################################################
# Install
###############################################################################

install(
  FILES
    find_lane.h
    road_builder.h
  DESTINATION
    ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/maliput
)
