# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")
load("@//tools:qt.bzl", "qt_moc_gen", "qt_rcc_gen")

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
        "IGN_GUI_PLUGIN_INSTALL_PATH=ign-gui-0/plugins",
    ],
)

iface_header = "include/ignition/gui/Iface.hh"
mainwindow_header = "include/ignition/gui/MainWindow.hh"
plugin_header = "include/ignition/gui/Plugin.hh"

public_headers = [
    "include/ignition/gui/ign.hh",
    "include/ignition/gui/plugins/ImageDisplay.hh",
    "include/ignition/gui/plugins/Publisher.hh",
    "include/ignition/gui/plugins/Requester.hh",
    "include/ignition/gui/plugins/Responder.hh",
    "include/ignition/gui/plugins/TimePanel.hh",
    "include/ignition/gui/plugins/TopicEcho.hh",
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
        # FIXME(clalancette): I removed the building of the plugin files here,
        # because it was incorrect.  We should be building those plugin files
        # as actual plugins (i.e. shared objects), but I'll leave that for
        # another day since we don't need those plugins right now.
        "src/Plugin.cc",
        "qrc_resources.cc",  # from qt_rcc_gen above
        "moc_iface.cc",  # from qt_moc_gen above
        "moc_mainwindow.cc",  # from qt_moc_gen above
        "moc_plugin.cc",  # from qt_moc_gen above
        "@ignition_common//:libignition_common.so",
    ],
    includes = ["include"],
    hdrs = public_headers + [iface_header, mainwindow_header, plugin_header],
    deps = [
        "@gts",
        "@ignition_common//:ignition_common_shared_library",
        "@ignition-transport3",
        "@Qt5Core",
        "@Qt5Gui",
        "@Qt5Widgets",
        "@tinyxml2",
    ],
    linkopts = [
        # clalancette: Unfortunately, libfreeimage-dev doesn't have a pkg-config
        # file on Ubuntu, so we hand-link this here.
        "-lfreeimage",
    ],
    linkstatic = 1,
)

cc_binary(
    name = "libignition_gui.so",
    visibility = ["//visibility:public"],
    linkshared = 1,
    linkstatic = 1,
    deps = [
        ":_libignition_gui.so",
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
