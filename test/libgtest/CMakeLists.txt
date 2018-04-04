##############################################################################
# Build GTest
##############################################################################

add_library(gtest gtest-1.7.0/src/gtest-all.cc)
target_link_libraries(gtest pthread)
target_include_directories(
  gtest
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/libgtest/gtest-1.7.0/include>
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/libgtest/gtest-1.7.0>
)
add_library(gtest_main gtest-1.7.0/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
target_include_directories(
  gtest_main
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/libgtest/gtest-1.7.0/include>
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/libgtest/gtest-1.7.0>
)

