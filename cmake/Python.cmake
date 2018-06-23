##############################################################################
# Python Setup
##############################################################################

# Pin down python 2.7
find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)

# Unfortunately no easy way to get site-packages / dist-packages
set(PYTHON_INSTALL_DIR lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)

if(NOT PYBIND11_PYTHON_VERSION)
  set(PYBIND11_PYTHON_VERSION "2.7" CACHE STRING "Python version to use for compiling modules")
endif()
find_package(pybind11 REQUIRED)
