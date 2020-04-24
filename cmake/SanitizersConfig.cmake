# Address Sanitizer Configuration
option(ADDRESS_SANITIZER "Enable Clang Address Sanitizer" OFF)
if (ADDRESS_SANITIZER)
    message(STATUS "Address Sanitizer - enabled")
    # Same flags when using either clang or gcc compilers.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope")
    set(LDFLAGS "${LDFLAGS} -fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope")
    set(SANITIZERS on)
else()
    message(STATUS "Address Sanitizer - disabled")
endif()
