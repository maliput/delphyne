##############################################################################
# Python Setup
##############################################################################

# Pin down python 3.6
find_package(PythonLibs 3.6 REQUIRED)
find_package(PythonInterp 3.6 REQUIRED)

# Unfortunately no easy way to get site-packages / dist-packages
set(PYTHON_INSTALL_DIR lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)

if(NOT PYBIND11_PYTHON_VERSION)
  set(PYBIND11_PYTHON_VERSION "3.6" CACHE STRING "Python version to use for compiling modules")
endif()
find_package(pybind11 REQUIRED)
