link_directories(${CMAKE_INSTALL_PREFIX}/lib)

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
