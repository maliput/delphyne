link_directories(${CMAKE_INSTALL_PREFIX}/lib)

# Important note:
# Loadable modules must *not* link against libdrake.so.
# If they do, they would have symbols defined twice; once in the loading program,
# and once in the loadable module. That is not OK for global read/write data,
# because when the original program starts up, it initializes that global
# read/write data (by either doing simple zero-initialization for simple types,
# or calling the constructor for complex types). If the loadable module *also*
# has that linked in, then when the loadable module gets loaded, its initialization
# is run again and the previously initialized variables are re-written.

get_target_property(STX_INCLUDE_DIRS stx::stx INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(DRAKE_LCMTYPES_INCLUDE_DIRS drake::drake-lcmtypes-cpp INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(DRAKE_INCLUDE_DIRS drake::drake INTERFACE_INCLUDE_DIRECTORIES)

############## LoadablePriusSimpleCar #####################
add_library(LoadablePriusSimpleCar SHARED
  LoadablePriusSimpleCar.cc
)

target_include_directories(LoadablePriusSimpleCar PRIVATE
  ${DRAKE_INCLUDE_DIRS}
  ${DRAKE_LCMTYPES_INCLUDE_DIRS}
  ${STX_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${DELPHYNE_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
)

install(TARGETS LoadablePriusSimpleCar DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents COMPONENT shlib)

############## LoadablePriusTrajectoryCar #####################
add_library(LoadablePriusTrajectoryCar SHARED
  LoadablePriusTrajectoryCar.cc
)

target_include_directories(LoadablePriusTrajectoryCar PRIVATE
  ${DRAKE_INCLUDE_DIRS}
  ${DRAKE_LCMTYPES_INCLUDE_DIRS}
  ${STX_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${DELPHYNE_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
)

install(TARGETS LoadablePriusTrajectoryCar DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents COMPONENT shlib)

############## LoadableMobilControlledSimpleCar #####################
add_library(LoadableMobilControlledSimpleCar SHARED
  LoadableMobilControlledSimpleCar.cc
)

target_include_directories(LoadableMobilControlledSimpleCar PRIVATE
  ${DRAKE_INCLUDE_DIRS}
  ${DRAKE_LCMTYPES_INCLUDE_DIRS}
  ${STX_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${DELPHYNE_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
)

install(TARGETS LoadableMobilControlledSimpleCar DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents COMPONENT shlib)
