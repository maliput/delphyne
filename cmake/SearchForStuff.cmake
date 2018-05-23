include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

########################################
# Find drake
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

########################################
# Find ignition common
find_package(ignition-common2 REQUIRED)
if (NOT ignition-common2_FOUND)
  message(STATUS "Looking for ignition-common2-config.cmake - not found")
  BUILD_ERROR ("Missing: Ignition common2 library (libignition-common2-dev).")
else()
  message(STATUS "Looking for ignition-common2-config.cmake - found")
  include_directories(${IGNITION-COMMON_INCLUDE_DIRS})
  link_directories(${IGNITION-COMMON_LIBRARY_DIRS})
endif()

########################################
# Find ignition math
find_package(ignition-math5 REQUIRED)
if (NOT ignition-math5_FOUND)
  message(STATUS "Looking for ignition-math5-config.cmake - not found")
  BUILD_ERROR ("Missing: Ignition math5 library (libignition-math5-dev).")
else()
  message(STATUS "Looking for ignition-math5-config.cmake - found")
  include_directories(${IGNITION-MATH_INCLUDE_DIRS})
  link_directories(${IGNITION-MATH_LIBRARY_DIRS})
endif()

########################################
# Find ignition msgs
find_package(ignition-msgs2 QUIET)
if (NOT ignition-msgs2_FOUND)
  message(STATUS "Looking for ignition-msgs2-config.cmake - not found")
  BUILD_ERROR ("Missing: Ignition msgs2 library (libignition-msgs2-dev).")
else()
  message(STATUS "Looking for ignition-msgs2-config.cmake - found")
  include_directories(${IGNITION-MSGS_INCLUDE_DIRS})
  link_directories(${IGNITION-MSGS_LIBRARY_DIRS})
endif()

########################################
# Find ignition transport
find_package(ignition-transport5 REQUIRED)
if (NOT ignition-transport5_FOUND)
  message(STATUS "Looking for ignition-transport5-config.cmake - not found")
  BUILD_ERROR ("Missing: Ignition transport5 library (libignition-transport5-dev).")
else()
  message(STATUS "Looking for ignition-transport5-config.cmake - found")
  include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
  link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})
endif()

########################################
# Find lcm
find_package(lcm REQUIRED)
if (NOT lcm_FOUND)
  message(STATUS "Looking for lcm-config.cmake - not found")
  BUILD_ERROR ("Missing: lcm library.")
else()
  message(STATUS "Looking for lcm-config.cmake - found")
  include_directories(${LCM_INCLUDE_DIRS})
  link_directories(${LCM_LIBRARY_DIRS})
endif()

