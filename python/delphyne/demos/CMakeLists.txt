##############################################################################
# Modules
##############################################################################

install(
  FILES
    __init__.py
    dragway.py
    helpers.py
    gazoo.py
    keyop.py
    realtime.py
    roads.py
    scriptlets.py
  DESTINATION
    ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/demos
)
