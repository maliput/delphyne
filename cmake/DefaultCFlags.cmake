# Note that we have to put '-rdynamic' into all of the LINK and C_FLAGS so that
# we get our symbols placed into the dynamic symbol table.  This is needed so
# that our loadable agents have access to the internal symbols from our library.

# Build type link flags
set (CMAKE_LINK_FLAGS_RELEASE " -rdynamic" CACHE INTERNAL "Link flags for release" FORCE)
set (CMAKE_LINK_FLAGS_RELWITHDEBINFO " -rdynamic" CACHE INTERNAL "Link flags for release with debug support" FORCE)
set (CMAKE_LINK_FLAGS_DEBUG " -rdynamic" CACHE INTERNAL "Link flags for debug" FORCE)
set (CMAKE_LINK_FLAGS_PROFILE " -rdynamic -pg" CACHE INTERNAL "Link flags for profile" FORCE)
set (CMAKE_LINK_FLAGS_COVERAGE " -rdynamic --coverage" CACHE INTERNAL "Link flags for static code checking" FORCE)

set (CMAKE_C_FLAGS_RELEASE "")
if (NOT APPLE)
  # -s doesn't work with default osx compiler clang, alternative:
  # http://stackoverflow.com/questions/6085491/gcc-vs-clang-symbol-strippingu
  set (CMAKE_C_FLAGS_RELEASE "-s")
endif()


set (CMAKE_C_FLAGS_RELEASE " ${CMAKE_C_FLAGS_RELEASE} -rdynamic -O3 -DNDEBUG ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release" FORCE)
set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})

set (CMAKE_C_FLAGS_RELWITHDEBINFO " -rdynamic -g -O2 ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release with debug support" FORCE)
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO ${CMAKE_C_FLAGS_RELWITHDEBINFO})

set (CMAKE_C_FLAGS_DEBUG " -rdynamic -ggdb3 ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for debug" FORCE)
set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -rdynamic")

set (CMAKE_C_FLAGS_PROFILE " -rdynamic -fno-omit-frame-pointer -g -pg ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for profile" FORCE)
set (CMAKE_CXX_FLAGS_PROFILE ${CMAKE_C_FLAGS_PROFILE})

set (CMAKE_C_FLAGS_COVERAGE " -rdynamic -g -O0 -Wformat=2 --coverage -fno-inline ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for static code checking" FORCE)
set (CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_C_FLAGS_COVERAGE}")
foreach(flag
        -fno-default-inline
        -fno-elide-constructors
        -fno-implicit-inline-templates)
  CHECK_CXX_COMPILER_FLAG(${flag} R${flag})
  if (R${flag})
    set (CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_COVERAGE} ${flag}")
  endif()
endforeach()

#####################################
# Set all the global build flags
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}} -std=c++14")

if (UNIX)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
    set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
    set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
    # Add visibility in UNIX
    check_gcc_visibility()
    if (GCC_SUPPORTS_VISIBILITY)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
    endif()
endif()

# Compiler-specific C++14 activation.
if ("${CMAKE_CXX_COMPILER_ID} " MATCHES "GNU ")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (NOT (GCC_VERSION VERSION_GREATER 5.2))
        message(FATAL_ERROR "${PROJECT_NAME} requires g++ 5.2 or greater.")
    endif ()
elseif ("${CMAKE_CXX_COMPILER_ID} " MATCHES "Clang ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
elseif ("${CMAKE_CXX_COMPILER_ID} " STREQUAL "MSVC ")
    if (NOT MSVC14)
        message(FATAL_ERROR "${PROJECT_NAME} requires VS 2013 os greater.")
    endif()
else ()
    message(FATAL_ERROR "Your C++ compiler does not support C++14.")
endif ()
