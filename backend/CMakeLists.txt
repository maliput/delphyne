include (${project_cmake_dir}/Utils.cmake)

add_executable(automotive-demo
  automotive_demo.cc
)

link_directories(${DRAKE_LIB_PREFIX})
target_link_libraries(automotive-demo
  Eigen3::Eigen
  stx::stx
  drake::drake-lcmtypes-cpp
  ${DRAKE_LIB_PREFIX}/libdrake.so
)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
