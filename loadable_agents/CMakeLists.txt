# Important note:
# Check the Delphyne Guide for details about linking with Drake libraries from
# any loadable agent.

############## LoadablePriusSimpleCar #####################
add_library(LoadablePriusSimpleCar
  LoadablePriusSimpleCar.cc
)

target_include_directories(
  LoadablePriusSimpleCar
  PRIVATE
    ${EIGEN_INCLUDE_DIRS}
    ${DELPHYNE_INCLUDE_DIRS}
)

target_link_libraries(
  LoadablePriusSimpleCar
  PRIVATE
    drake::drake
    ignition-common1::ignition-common1
    stx::stx
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
    ${EIGEN_INCLUDE_DIRS}
    ${DELPHYNE_INCLUDE_DIRS}
)

target_link_libraries(
  LoadablePriusTrajectoryCar
  PRIVATE
    drake::drake
    ignition-common1::ignition-common1
    stx::stx
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
    ${EIGEN_INCLUDE_DIRS}
    ${DELPHYNE_INCLUDE_DIRS}
)

target_link_libraries(
  LoadableMobilControlledSimpleCar
  PRIVATE
    drake::drake
    ignition-common1::ignition-common1
    stx::stx
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
    ${EIGEN_INCLUDE_DIRS}
    ${DELPHYNE_INCLUDE_DIRS}
)

target_link_libraries(
  LoadableMaliputRailCar
  PRIVATE
    drake::drake
    ignition-common1::ignition-common1
    stx::stx
)

install(TARGETS LoadableMaliputRailCar
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents)
