cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project (delphyne)

string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set (PROJECT_MAJOR_VERSION 0)
set (PROJECT_MINOR_VERSION 1)
set (PROJECT_PATCH_VERSION 1)

set (PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION})
set (PROJECT_VERSION_FULL ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION_FULL}")

set (project_cmake_dir ${PROJECT_SOURCE_DIR}/cmake
  CACHE PATH "Location of CMake scripts")

include (${project_cmake_dir}/Utils.cmake)

# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

# developer's option to cache PKG_CONFIG_PATH and
# LD_LIBRARY_PATH for local installs
if(PKG_CONFIG_PATH)
  set (ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
endif()
if(LD_LIBRARY_PATH)
  set (ENV{LD_LIBRARY_PATH} ${LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH})
endif()

set (USE_FULL_RPATH OFF CACHE BOOL "Set to true to enable full rpath")

if (USE_FULL_RPATH)
  # use, i.e. don't skip the full RPATH for the build tree
  set(CMAKE_SKIP_BUILD_RPATH  FALSE)

  # when building, don't use the install RPATH already
  # (but later on when installing)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")

  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  # the RPATH to be used when installing, but only if its not a system directory
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
  endif("${isSystemDir}" STREQUAL "-1")
endif()

set (BUILD_DELPHYNE ON CACHE INTERNAL
  "Build Delphyne" FORCE)
set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)

include (${project_cmake_dir}/DissectVersion.cmake)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message (STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(drake REQUIRED)
find_package(ignition-common2 REQUIRED)
find_package(ignition-math5 REQUIRED)
find_package(ignition-msgs2 REQUIRED)
find_package(ignition-transport5 REQUIRED)

message (STATUS "\n----------------------------------------\n")

#####################################
MESSAGE(STATUS "Checking build type")
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo Profile Check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
# TODO: still convert to uppercase to keep backwards compatibility with
# uppercase old supported and deprecated modes
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set (BUILD_TYPE_PROFILE FALSE)
set (BUILD_TYPE_RELEASE FALSE)
set (BUILD_TYPE_RELWITHDEBINFO FALSE)
set (BUILD_TYPE_DEBUG FALSE)

if ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "PROFILE")
  set (BUILD_TYPE_PROFILE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set (BUILD_TYPE_RELEASE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set (BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set (BUILD_TYPE_DEBUG TRUE)
else()
  build_error("CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug Release RelWithDebInfo Profile Check")
endif()

#####################################
# Handle CFlags
unset (CMAKE_C_FLAGS_ALL CACHE)
unset (CMAKE_CXX_FLAGS CACHE)

# USE_HOST_CFLAGS (default TRUE)
# Will check building host machine for proper cflags
if(NOT DEFINED USE_HOST_CFLAGS OR USE_HOST_CFLAGS)
  message(STATUS "Enable host CFlags")
  include (${project_cmake_dir}/HostCFlags.cmake)
endif()

# USE_UPSTREAM_CFLAGS (default TRUE)
# Will use predefined delphyne developers cflags
if(NOT DEFINED USE_UPSTREAM_CFLAGS OR USE_UPSTREAM_CFLAGS)
  message(STATUS "Enable upstream CFlags")
  include(${project_cmake_dir}/DefaultCFlags.cmake)
endif()

# Check if warning options are avaliable for the compiler and return WARNING_CXX_FLAGS variable
list(APPEND WARN_LEVEL -Waddress -Warray-bounds -Wcomment -Wformat -Wnonnull)
list(APPEND WARN_LEVEL -Wparentheses -Wreorder -Wreturn-type)
list(APPEND WARN_LEVEL -Wsequence-point -Wsign-compare -Wstrict-aliasing)
list(APPEND WARN_LEVEL -Wstrict-overflow=1 -Wswitch -Wtrigraphs -Wuninitialized)
list(APPEND WARN_LEVEL -Wunused-function -Wunused-label -Wunused-value)
list(APPEND WARN_LEVEL -Wunused-variable -Wvolatile-register-var)

# Unable to be filtered flags (failing due to limitations in filter_valid_compiler_warnings)
set(UNFILTERED_FLAGS "-Wc++14-compat")

filter_valid_compiler_warnings(${WARN_LEVEL} -Wextra -Wno-long-long
  -Wno-unused-value -Wno-unused-value -Wno-unused-value -Wno-unused-value
  -Winit-self -Wswitch-default
  -Wmissing-include-dirs -Wno-pragmas)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}${WARNING_CXX_FLAGS} ${UNFILTERED_FLAGS}")

set(BUILD_SHARED_LIBS TRUE)

#################################################
# Print warnings and errors
if ( build_warnings )
  message(STATUS "BUILD WARNINGS")
  foreach (msg ${build_warnings})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD WARNINGS\n")
endif (build_warnings)

########### Add uninstall target ###############
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")

if (build_errors)
  message(STATUS "BUILD ERRORS: These must be resolved before compiling.")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD ERRORS\n")
  message (FATAL_ERROR "Errors encountered in build. "
      "Please see the BUILD ERRORS above.")
else (build_errors)

  ########################################
  # Write the config.hh file
  configure_file (${project_cmake_dir}/config.hh.in
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/config.hh)
  delphyne_install_includes(
    ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/config.hh)

  include_directories(
    ${CMAKE_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
    ${PROJECT_BINARY_DIR}/include
  )

  link_directories(${PROJECT_BINARY_DIR}/src)

  if (DEFINED CMAKE_CXX_FLAGS)
    message (STATUS "Custom CFlags:${CMAKE_CXX_FLAGS}")
  else()
    message (STATUS "Use default CFlags")
  endif()
  message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
  message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")

  add_subdirectory(include)
  add_subdirectory(src)
  add_subdirectory(python)

  ########################################
  # Testing
  if(NOT DEFINED BUILD_TESTING OR BUILD_TESTING)
    enable_testing()
    add_subdirectory(test)
  endif()

  ########################################
  # Make the cmake config files

  include(CMakePackageConfigHelpers)


  ########################################
  # Package Preparation

  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake
  )

  # Version module
  write_basic_config_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION_FULL}
    COMPATIBILITY SameMajorVersion
  )

  # Install config, version and target modules
  install(
    FILES
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    DESTINATION
      ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}
  )
  install(
    EXPORT ${PROJECT_NAME}-targets
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION
      ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}
  )

  ########################################
  # If present, load platform-specific build hooks.  This system is used,
  # for example, by the Ubuntu overlay, to arrange for installation of
  # Ubuntu-specific application-launching configuration.
  if (EXISTS ${PROJECT_SOURCE_DIR}/cmake/packager-hooks/CMakeLists.txt)
    message(STATUS "Loading packager build hooks from cmake/packager-hooks")
    add_subdirectory(cmake/packager-hooks)
  endif()

  message(STATUS "Configuration successful. Type make to compile
  ${PROJECT_NAME_LOWER}")
endif(build_errors)
