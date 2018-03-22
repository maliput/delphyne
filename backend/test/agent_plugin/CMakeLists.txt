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

add_library(LoadableExampleDouble SHARED
  LoadableExampleDouble.cc
)

target_include_directories(LoadableExampleDouble PRIVATE
  ${DRAKE_LCMTYPES_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${STX_INCLUDE_DIRS}
)

add_library(LoadableExampleAutoDiffXd SHARED
  LoadableExampleAutoDiffXd.cc
)

target_include_directories(LoadableExampleAutoDiffXd PRIVATE
  ${DRAKE_LCMTYPES_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${STX_INCLUDE_DIRS}
)

add_library(LoadableExampleExpression SHARED
  LoadableExampleExpression.cc
)

target_include_directories(LoadableExampleExpression PRIVATE
  ${DRAKE_LCMTYPES_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${STX_INCLUDE_DIRS}
)
