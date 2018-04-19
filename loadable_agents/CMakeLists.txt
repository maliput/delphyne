# Important note:
# Check the Delphyne Guide for details about linking with Drake libraries from
# any loadable agent.

get_target_property(STX_INCLUDE_DIRS stx::stx INTERFACE_INCLUDE_DIRECTORIES)

############## LoadablePriusSimpleCar #####################
add_library(LoadablePriusSimpleCar
  LoadablePriusSimpleCar.cc
)

target_include_directories(
  LoadablePriusSimpleCar
  PRIVATE
    ${STX_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${DELPHYNE_INCLUDE_DIRS}
)

target_link_libraries(
  LoadablePriusSimpleCar
  PRIVATE
    drake::drake
    ignition-common1::ignition-common1
)

install(TARGETS LoadablePriusSimpleCar
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents)

############## LoadablePriusTrajectoryCar #####################
add_library(LoadablePriusTrajectoryCar
  LoadablePriusTrajectoryCar.cc
)

target_include_directories(
  LoadablePriusTrajectoryCar
  PRIVATE
    ${STX_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${DELPHYNE_INCLUDE_DIRS}
)

target_link_libraries(
  LoadablePriusTrajectoryCar
  PRIVATE
    drake::drake
    ignition-common1::ignition-common1
)

install(TARGETS LoadablePriusTrajectoryCar
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents)

############## LoadableMobilControlledSimpleCar #####################
add_library(LoadableMobilControlledSimpleCar
  LoadableMobilControlledSimpleCar.cc
)

target_include_directories(
  LoadableMobilControlledSimpleCar
  PRIVATE
    ${STX_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${DELPHYNE_INCLUDE_DIRS}
)

target_link_libraries(
  LoadableMobilControlledSimpleCar
  PRIVATE
    drake::drake
    ignition-common1::ignition-common1
)

install(TARGETS LoadableMobilControlledSimpleCar
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents)

############## LoadableMaliputRailCar #####################
add_library(LoadableMaliputRailCar SHARED
  LoadableMaliputRailCar.cc
)

target_include_directories(
  LoadableMaliputRailCar
  PRIVATE
    ${STX_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${DELPHYNE_INCLUDE_DIRS}
)

target_link_libraries(
  LoadableMaliputRailCar
  PRIVATE
    drake::drake
    ignition-common1::ignition-common1
)

install(TARGETS LoadableMaliputRailCar
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents)
