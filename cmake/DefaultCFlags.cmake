#####################################
# Handle CFlags

unset (CMAKE_CXX_FLAGS CACHE)

set (CMAKE_C_FLAGS_RELWITHDEBINFO " -g -O2" CACHE INTERNAL "C Flags for release with debug support" FORCE)
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO ${CMAKE_C_FLAGS_RELWITHDEBINFO})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}} -std=c++17")

# C++ Version
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

# Compiler-specific C++17 activation.
if ("${CMAKE_CXX_COMPILER_ID} " MATCHES "GNU ")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (NOT (GCC_VERSION VERSION_GREATER 6.9))
        message(FATAL_ERROR "${PROJECT_NAME} requires g++ 7.0 or greater.")
    else ()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdata-sections -fdiagnostics-color=always -ffunction-sections -fopenmp -fPIC   -fstack-protector -fno-omit-frame-pointer -no-canonical-prefixes -O2 -std=c++17 -Wall -Wno-builtin-macro-redefined -Wno-missing-field-initializers -Wregister -Wstrict-overflow -Wno-unused-const-variable")
    endif ()
elseif ("${CMAKE_CXX_COMPILER_ID} " MATCHES "Clang ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdata-sections -fdiagnostics-color=always -ffunction-sections -fPIC -fstack-protector -fno-omit-frame-pointer -no-canonical-prefixes -O2 -std=c++17 -Wall -Wno-builtin-macro-redefined -Wno-deprecated-dynamic-exception-spec -Wno-enum-compare-switch -Wno-gnu-designator -Wno-missing-field-initializers -Wno-register -Wno-strict-overflow -Wno-unknown-warning-option -Wno-unneeded-internal-declaration -Wno-unused-const-variable -Wno-missing-braces")
else ()
    message(FATAL_ERROR "Your C++ compiler does not support C++17.")
endif ()

# Check if warning options are avaliable for the compiler and return WARNING_CXX_FLAGS variable
list(APPEND WARN_LEVEL -Waddress -Warray-bounds -Wcomment -Wformat -Wnonnull)
list(APPEND WARN_LEVEL -Wparentheses -Wreorder -Wreturn-type)
list(APPEND WARN_LEVEL -Wsequence-point -Wsign-compare -Wstrict-aliasing)
list(APPEND WARN_LEVEL -Wstrict-overflow=1 -Wswitch -Wtrigraphs -Wuninitialized)
list(APPEND WARN_LEVEL -Wunused-function -Wunused-label -Wunused-value)
list(APPEND WARN_LEVEL -Wunused-variable -Wvolatile-register-var)

# Unable to be filtered flags (failing due to limitations in filter_valid_compiler_warnings)
set(UNFILTERED_FLAGS "-Wc++17-compat")

filter_valid_compiler_warnings(${WARN_LEVEL} -Wextra -Wno-long-long
  -Wno-unused-value -Wno-unused-value -Wno-unused-value -Wno-unused-value
  -Winit-self -Wswitch-default
  -Wmissing-include-dirs -Wno-pragmas)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}${WARNING_CXX_FLAGS} ${UNFILTERED_FLAGS}")
