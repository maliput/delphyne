##############################################################################
# Build GTest
##############################################################################

add_library(gtest gtest/src/gtest-all.cc)
target_link_libraries(gtest pthread)
add_library(gtest_main gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)

##############################################################################
# Tests
##############################################################################

include_directories (
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
  ${PROJECT_BINARY_DIR}
)

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/include/delphyne/test_config.h)

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
