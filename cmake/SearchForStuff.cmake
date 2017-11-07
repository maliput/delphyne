include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

# Append install_drake path to CMAKE_PREFIX_PATH to enable
# find_package to find drake-related .cmake files
list(APPEND CMAKE_PREFIX_PATH ${DRAKE_INSTALL_PREFIX})

########################################
# Find drake in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(drake REQUIRED)
  if (NOT drake_FOUND)
    message(STATUS "Looking for drake-config.cmake - not found")
    BUILD_ERROR ("Missing: Drake library (libdrake).")
  else()
    message(STATUS "Looking for drake-config.cmake - found")
    include_directories(${DRAKE_INCLUDE_DIRS})
    link_directories(${DRAKE_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find ignition common in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-common0 0.4 QUIET)
  if (NOT ignition-common0_FOUND)
    message(STATUS "Looking for ignition-common0-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition common0 library (libignition-common0-dev).")
  else()
    message(STATUS "Looking for ignition-common0-config.cmake - found")
    include_directories(${IGNITION-COMMON_INCLUDE_DIRS})
    link_directories(${IGNITION-COMMON_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find ignition math in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-math3 REQUIRED)
  if (NOT ignition-math3_FOUND)
    message(STATUS "Looking for ignition-math3-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition math3 library (libignition-math3-dev).")
  else()
    message(STATUS "Looking for ignition-math3-config.cmake - found")
    include_directories(${IGNITION-MATH_INCLUDE_DIRS})
    link_directories(${IGNITION-MATH_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find ignition msgs in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-msgs1 QUIET)
  if (NOT ignition-msgs1_FOUND)
    message(STATUS "Looking for ignition-msgs1-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition msgs1 library (libignition-msgs1-dev).")
  else()
    message(STATUS "Looking for ignition-msgs1-config.cmake - found")
    include_directories(${IGNITION-MSGS_INCLUDE_DIRS})
    link_directories(${IGNITION-MSGS_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find ignition transport in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-transport3 REQUIRED)
  if (NOT ignition-transport3_FOUND)
    message(STATUS "Looking for ignition-transport3-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition transport3 library (libignition-transport3-dev).")
  else()
    message(STATUS "Looking for ignition-transport3-config.cmake - found")
    include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
    link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})
  endif()
endif()

<<<<<<< c1857b94a810a950ef16edd8cdc1e2d69bbfc40a
=======

>>>>>>> Fixed issues with cmake imports
########################################
# Find gflags in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(gflags REQUIRED)
  if (NOT gflags_FOUND)
    message(STATUS "Looking for gflags-config.cmake - not found")
    BUILD_ERROR ("Missing: Gflags library (libgflags).")
  else()
    message(STATUS "Looking for gflags-config.cmake - found")
    include_directories(${GFLAGS_INCLUDE_DIRS})
    link_directories(${GFLAGS_LIBRARY_DIRS})
  endif()
endif()


########################################
# Find lcm in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(lcm REQUIRED)
  if (NOT lcm_FOUND)
    message(STATUS "Looking for lcm-config.cmake - not found")
    BUILD_ERROR ("Missing: lcm library.")
  else()
    message(STATUS "Looking for lcm-config.cmake - found")
    include_directories(${LCM_INCLUDE_DIRS})
    link_directories(${LCM_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find QT
find_package (Qt5Widgets)
if (NOT Qt5Widgets_FOUND)
  BUILD_ERROR("Missing: Qt5Widgets")
endif()

find_package (Qt5Core)
if (NOT Qt5Core_FOUND)
  BUILD_ERROR("Missing: Qt5Core")
endif()

find_package (Qt5OpenGL)
if (NOT Qt5OpenGL_FOUND)
  BUILD_ERROR("Missing: Qt5OpenGL")
endif()

find_package (Qt5Test)
if (NOT Qt5Test_FOUND)
  BUILD_ERROR("Missing: Qt5Test")
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()
