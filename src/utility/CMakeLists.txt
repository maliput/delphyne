##############################################################################
# Delphyne utility
##############################################################################

add_library(utility
  package.cc
)

set_target_properties(utility PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-utility)
target_include_directories(
  utility
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
  
target_link_libraries(utility
  PUBLIC
    public_headers
    ${IGNITION-COMMON_LIBRARIES}
)

install(TARGETS utility EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
