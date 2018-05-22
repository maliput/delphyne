##############################################################################
# Python Module
##############################################################################

find_program(PYTHON "python")

if (PYTHON)
    set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in")
    set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
    set(OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/build/delphyne")

    configure_file(${SETUP_PY_IN} ${SETUP_PY})

    add_custom_command(OUTPUT ${OUTPUT}
                       COMMAND ${PYTHON} ${SETUP_PY} build
                       COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT})

    add_custom_target(target ALL DEPENDS ${OUTPUT})
    # This will set the required PYTHONPATH variables when executing python setup.py.
    # The delphyne.egg.info dir was set to be created on the build directory in order
    # to avoid setuptools from spamming the source dir.
    install(CODE "set(ENV{PYTHONPATH} ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_LIBRARY_SUFFIX}/site-packages:${CMAKE_INSTALL_PREFIX}/lib)"
            CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} egg_info --egg-base ${CMAKE_CURRENT_BINARY_DIR} sdist install --prefix=${CMAKE_INSTALL_PREFIX})")
endif()
