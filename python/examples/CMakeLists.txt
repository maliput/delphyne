##############################################################################
# Scripts
##############################################################################

# TODO(daniel.stonier) if we ever have other executables that are not demos
# consider renaming these to delphyne-demo-<name>

install(
  PROGRAMS
    delphyne-crash
    delphyne-dragway
    delphyne-gazoo
    delphyne-keyop
    delphyne-perf
    delphyne-realtime
    delphyne-roads
    delphyne-scriptlets
  DESTINATION
    bin
)
