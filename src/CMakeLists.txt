##############################################################################
# Headers
##############################################################################

# Not fond of this as it's a level of indirection and makes accessible
# headers that shouldn't be exposed to one another, but cpplint needs
# it to find private headers sitting # alongside their .cc files (relative
# lookups not allowed).
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


##############################################################################
# Directories
##############################################################################

add_subdirectory(agents)
add_subdirectory(systems)
