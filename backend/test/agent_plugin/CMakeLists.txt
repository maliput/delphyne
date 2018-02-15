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
