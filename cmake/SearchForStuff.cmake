include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

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
  find_package(ignition-math4 REQUIRED)
  if (NOT ignition-math4_FOUND)
    message(STATUS "Looking for ignition-math4-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition math4 library (libignition-math4-dev).")
  else()
    message(STATUS "Looking for ignition-math4-config.cmake - found")
    include_directories(${IGNITION-MATH_INCLUDE_DIRS})
    link_directories(${IGNITION-MATH_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find ignition msgs in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-msgs0 QUIET)
  if (NOT ignition-msgs0_FOUND)
    message(STATUS "Looking for ignition-msgs0-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition msgs0 library (libignition-msgs0-dev).")
  else()
    message(STATUS "Looking for ignition-msgs0-config.cmake - found")
    include_directories(${IGNITION-MSGS_INCLUDE_DIRS})
    link_directories(${IGNITION-MSGS_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find ignition gui in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-gui0 0.1 QUIET)
  if (NOT ignition-gui0_FOUND)
    message(STATUS "Looking for ignition-gui0-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition gui0 library (libignition-gui0-dev).")
  else()
    message(STATUS "Looking for ignition-gui0-config.cmake - found")
    include_directories(${IGNITION-GUI_INCLUDE_DIRS})
    link_directories(${IGNITION-GUI_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find ignition rendering in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-rendering0 0.1 QUIET)
  if (NOT ignition-rendering0_FOUND)
    message(STATUS "Looking for ignition-rendering0-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition rendering0 library (libignition-rendering0-dev).")
  else()
    message(STATUS "Looking for ignition-rendering0-config.cmake - found")
    include_directories(${IGNITION-RENDERING_INCLUDE_DIRS})
    MESSAGE(STATUS "rendering include: ${IGNITION-RENDERING_INCLUDE_DIRS}")
    link_directories(${IGNITION-RENDERING_LIBRARY_DIRS})
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

########################################
# Find lcm in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(lcm REQUIRED)
  if (NOT lcm_FOUND)
    message(STATUS "Looking for lcmConfig.cmake - not found")
    BUILD_ERROR ("Missing: lcm library.")
  else()
    message(STATUS "Looking for lcmConfig.cmake - found")
  endif()
endif()

########################################
# Find lcm

#pkg_check_modules(lcm lcm)
#if (NOT lcm_FOUND)
#  message(STATUS "Looking for lcm.pc - not found")
#  BUILD_ERROR ("Missing: lcm library (liblcm-dev).")
#else()
#  message(STATUS "Looking for lcm.pc - found")
#  include_directories(${lcm_INCLUDEDIR})
#  link_directories(${lcm_LIBDIR})
#endif ()

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
