##############################################################################
# Replayer
##############################################################################

add_executable(replayer
  replayer.cc
)

target_link_libraries(replayer
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

install(TARGETS replayer
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
