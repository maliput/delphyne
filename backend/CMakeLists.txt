include (${project_cmake_dir}/Utils.cmake)

add_executable(automotive-demo
  automotive_demo.cc
)

target_link_libraries(automotive-demo
  ${drake_LIBRARIES}
)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
