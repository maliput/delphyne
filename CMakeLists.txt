##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(delphyne LANGUAGES C CXX VERSION 3.0.0)

##############################################################################
# CMake Support
##############################################################################

# Useful string/list macros
include (${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)

# Set CMAKE_INSTALL_<INCLUDEDIR|BINDIR|LIBDIR>
include(GNUInstallDirs)

##############################################################################
# Uninstall Target
##############################################################################

configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${PROJECT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P
  "${PROJECT_BINARY_DIR}/cmake/cmake_uninstall.cmake")

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(drake REQUIRED)
find_package(malidrive REQUIRED)
find_package(ignition-common2 REQUIRED COMPONENTS graphics)
find_package(ignition-math5 REQUIRED)
find_package(ignition-msgs2 REQUIRED)
find_package(ignition-transport5 REQUIRED COMPONENTS log)
pkg_check_modules(LIBZIP REQUIRED libzip)

##############################################################################
# Project Configuration
##############################################################################

message(STATUS "\n\n========= Project Configuration ========\n")

set(BUILD_SHARED_LIBS true)

# Ugh, makefile generators don't set any configuration (build) types. Use GNU's
set(DELPHYNE_CONFIGURATION_TYPES
      ${CMAKE_CONFIGURATION_TYPES} Debug MinSizeRel Release RelWithDebInfo)
list(REMOVE_DUPLICATES DELPHYNE_CONFIGURATION_TYPES)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
    STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    ${DELPHYNE_CONFIGURATION_TYPES})
endif()

if(NOT ${CMAKE_BUILD_TYPE} IN_LIST DELPHYNE_CONFIGURATION_TYPES)
  message(
    FATAL_ERROR
      "Build type '${CMAKE_BUILD_TYPE}' unknown. "
      "Valid options are: ${DELPHYNE_CONFIGURATION_TYPES}.")
endif()

# TODO: still convert to uppercase to keep backwards compatibility with
# uppercase old supported and deprecated modes
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

include(${PROJECT_SOURCE_DIR}/cmake/DefaultCFlags.cmake)

# Report out
message(STATUS "Shared Libraries..............${BUILD_SHARED_LIBS}")
message(STATUS "Build Type....................${CMAKE_BUILD_TYPE}")
message(STATUS "Install path..................${CMAKE_INSTALL_PREFIX}")
if (DEFINED CMAKE_CXX_FLAGS)
  message(STATUS "Custom CFlags.................${CMAKE_CXX_FLAGS}")
else()
  message (STATUS "Using default CFlags")
endif()

message(STATUS "\n----------------------------------------\n")

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(python)
add_subdirectory(media)

##############################################################################
# Tests
##############################################################################

# We always enable tests (that might change in the future)
enable_testing()
add_subdirectory(test)

##############################################################################
# Export
##############################################################################

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake
)

# Version module
write_basic_config_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install config, version and target modules
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)
install(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

configure_file(
  setup.bash.in
  ${PROJECT_BINARY_DIR}/setup.bash
  IMMEDIATE @ONLY
)
install(
  PROGRAMS ${PROJECT_BINARY_DIR}/setup.bash
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)
