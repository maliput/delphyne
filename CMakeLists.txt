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
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(drake_vendor REQUIRED)
find_package(malidrive REQUIRED)
find_package(maliput REQUIRED)
find_package(maliput-utilities REQUIRED)
find_package(multilane REQUIRED)
find_package(dragway REQUIRED)
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

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
  add_test(NAME CPPLINT COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_cpplint.sh)
  add_test(NAME PYLINT COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_python_checks.sh)
endif()

##############################################################################
# Export
##############################################################################

ament_environment_hooks(setup.bash.in)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(drake_vendor)
ament_export_dependencies(malidrive)
ament_export_dependencies(maliput)
ament_export_dependencies(multilane)
ament_export_dependencies(dragway)
ament_export_dependencies(maliput-utilities)
ament_export_interfaces(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

ament_package()
