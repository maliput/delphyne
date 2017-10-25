BASEDIR=$( realpath $( pwd ) )

export PKG_CONFIG_PATH=$BASEDIR/install/lib/pkgconfig:$BASEDIR/src/delphyne/backend/pkgconfig:$PKG_CONFIG_PATH
export PATH=$BASEDIR/install/bin:$PATH
export LD_LIBRARY_PATH=$BASEDIR/install/lib:$BASEDIR/install_drake/lib:$LD_LIBRARY_PATH
export DELPHYNE_PACKAGE_PATH=$BASEDIR/src/drake/drake/automotive/models:$BASEDIR/src/delphyne/bridge:$BASEDIR/src/delphyne:$DELPHYNE_PACKAGE_PATH
export DRAKE_INSTALL_PATH=$BASEDIR/install_drake
