include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)

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
    include_directories(${CMAKE_INSTALL_PREFIX}/include)
    include_directories(${CMAKE_INSTALL_PREFIX}/include/lcm)
    link_directories(${DRAKE_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find ignition common in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-common1 REQUIRED)
  if (NOT ignition-common1_FOUND)
    message(STATUS "Looking for ignition-common1-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition common0 library (libignition-common1-dev).")
  else()
    message(STATUS "Looking for ignition-common1-config.cmake - found")
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
    set(IGNITION-MATH_INCLUDE_DIRS ${IGNITION-MATH_INCLUDE_DIRS}/ignition-math4)
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
  find_package(ignition-transport4 REQUIRED)
  if (NOT ignition-transport4_FOUND)
    message(STATUS "Looking for ignition-transport4-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition transport4 library (libignition-transport4-dev).")
  else()
    message(STATUS "Looking for ignition-transport4-config.cmake - found")
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
    message(STATUS "Looking for lcm-config.cmake - not found")
    BUILD_ERROR ("Missing: lcm library.")
  else()
    message(STATUS "Looking for lcm-config.cmake - found")
    include_directories(${LCM_INCLUDE_DIRS})
    link_directories(${LCM_LIBRARY_DIRS})
  endif()
endif()

##########################################
# Find pybind and pin down python 2.7
find_package(PythonLibs 2.7)
if (NOT PythonLibs_FOUND)
  BUILD_ERROR("Missing: PythonLibs")
endif()

if(NOT PYBIND11_PYTHON_VERSION)
  set(PYBIND11_PYTHON_VERSION "2.7" CACHE STRING "Python version to use for compiling modules")
endif()
find_package(pybind11)
if (NOT pybind11_FOUND)
  BUILD_ERROR("Missing: pybind11")
endif()
# TODO(mikaelarguedas) uncomment this once we switch to python3
# if (NOT PYTHON_MODULE_EXTENSION MATCHES "cpython")
#   BUILD_ERROR("pybind didn't find a cpython interpreter: ${PYTHON_MODULE_EXTENSION}")
# endif()
