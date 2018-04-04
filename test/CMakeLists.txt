##############################################################################
# Build GTest
##############################################################################

add_library(gtest gtest/src/gtest-all.cc)
target_link_libraries(gtest pthread)
target_include_directories(
  gtest
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/gtest/include>
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/gtest>
)
add_library(gtest_main gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
target_include_directories(
  gtest_main
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/gtest/include>
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/gtest>
)

##############################################################################
# Tests
##############################################################################

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/include/delphyne/test_config.h)

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)

add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
