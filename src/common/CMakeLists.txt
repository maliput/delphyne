##############################################################################
# Library
##############################################################################

add_library(common
  compression.cc
  filesystem.cc
)

set_target_properties(common PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-common)
target_include_directories(
  common
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    ${LIBZIP_INCLUDE_DIRS}
)

target_link_libraries(
  common
  PUBLIC
    public_headers
    ignition-common2::ignition-common2
    ${LIBZIP_LIBRARIES}

)

install(TARGETS common EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
