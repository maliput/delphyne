# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")
load("@//tools:qt.bzl", "qt_moc_gen", "qt_rcc_gen")
load("@//tools:ign-gui.bzl", "ign_gui_create_plugins", "ign_gui_plugin_dep_name", "IGN_GUI_PLUGIN_PATH")

# Generates config.hh based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/config.hh.in",
    out = "include/ignition/gui/config.hh",
    cmakelists = ["CMakeLists.txt"],
    defines = [
        # It would be nice to get this information directly from CMakeLists.txt,
        # but it ends up being too hard.  ignition-gui sets a project name
        # as "ignition-gui<version>", and then uses CMake substring to pick
        # that version out.  We'd have to extend the cmake_configure_file
        # functionality to do the same, and I'm not sure it is worth it.  We
        # just hard code the major version here.
        "PROJECT_NAME=ignition-gui",
        "PROJECT_NAME_NO_VERSION=ignition-gui",
        "PROJECT_NAME_UPPER_NODASH=IGNITION_GUI",
        "IGN_GUI_PLUGIN_INSTALL_PATH=" + IGN_GUI_PLUGIN_PATH,
    ],
)

iface_header = "include/ignition/gui/Iface.hh"
mainwindow_header = "include/ignition/gui/MainWindow.hh"
plugin_header = "include/ignition/gui/Plugin.hh"

public_headers = [
    "include/ignition/gui/ign.hh",
    "include/ignition/gui/qt.h",
    "include/ignition/gui/System.hh",
]

# Generates gui.hh, which consists of #include statements for all of the
# public headers in the library.  The first line is
# '#include <ignition/gui/config.hh>' followed by one line like
# '#include <ignition/gui/Example.hh>' for each non-generated header.
genrule(
    name = "guihh_genrule",
    srcs = public_headers + [iface_header, mainwindow_header, plugin_header],
    outs = ["include/ignition/gui.hh"],
    # TODO: centralize this logic, as it is used here, in sdformat.BUILD, and
    # in fcl.BUILD
    cmd = "(" + (
        "echo '#include <ignition/gui/config.hh>' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g'"
    ) + ") > '$@'",
)

qt_moc_gen(
    name = "iface",
    hdr = iface_header,
    out = "moc_iface.cc",
)

qt_moc_gen(
    name = "mainwindow",
    hdr = mainwindow_header,
    out = "moc_mainwindow.cc",
)

qt_moc_gen(
    name = "plugin",
    hdr = plugin_header,
    out = "moc_plugin.cc",
)

qt_rcc_gen(
    name = "resources",
    qrc = "include/ignition/gui/resources.qrc",
    srcs = ["include/ignition/gui/style.qss", "include/ignition/gui/images/close.svg", "include/ignition/gui/images/undock.svg"],
    out = "qrc_resources.cc",
)

# Create all of the plugins here as separate shared objects.  The code
# to do this lives in ign-gui.bzl, since Bazel does not allow functions
# inside of BUILD files.
ign_gui_create_plugins(["ImageDisplay", "Publisher", "Requester", "Responder", "TimePanel", "TopicEcho"], public_headers)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources plus private headers.  The explicitly
# listed hdrs= matches upstream's public headers.
cc_library(
    name = "_libignition_gui.so",
    srcs = [
        "include/ignition/gui/config.hh",  # from cmake_configure_file above
        "include/ignition/gui.hh",
        "src/Iface.cc",
        "src/ign.cc",
        "src/MainWindow.cc",
        "src/Plugin.cc",
        "qrc_resources.cc",  # from qt_rcc_gen above
        "moc_iface.cc",  # from qt_moc_gen above
        "moc_mainwindow.cc",  # from qt_moc_gen above
        "moc_plugin.cc",  # from qt_moc_gen above
    ],
    deps = [
        "@gts",
        "@ignition_common//:ignition_common_shared_library",
        "@ignition-transport3",
        "@Qt5Core",
        "@Qt5Gui",
        "@Qt5Widgets",
        "@tinyxml2",
    ],
    hdrs = public_headers + [iface_header, mainwindow_header, plugin_header],
    includes = ["include"],
    linkopts = [
        # clalancette: Unfortunately, libfreeimage-dev doesn't have a pkg-config
        # file on Ubuntu, so we hand-link this here.
        "-lfreeimage",
    ],
    linkstatic = 0,
)

cc_binary(
    name = "libignition_gui.so",
    linkshared = 1,
    linkstatic = 0,
    deps = [
        ":_libignition_gui.so",
    ],
    data = [
        ign_gui_plugin_dep_name("ImageDisplay"),
        ign_gui_plugin_dep_name("Publisher"),
        ign_gui_plugin_dep_name("Requester"),
        ign_gui_plugin_dep_name("Responder"),
        ign_gui_plugin_dep_name("TimePanel"),
        ign_gui_plugin_dep_name("TopicEcho"),
    ],
)

cc_library(
    name = "ignition_gui_shared_library",
    visibility = ["//visibility:public"],
    deps = [
        ":_libignition_gui.so",
    ],
    data = [
        ":libignition_gui.so",
    ],
)
