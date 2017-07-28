# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

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

public_headers = [
    "include/ignition/gui/Iface.hh",
    "include/ignition/gui/ign.hh",
    "include/ignition/gui/MainWindow.hh",
    "include/ignition/gui/Plugin.hh",
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
    srcs = public_headers,
    outs = ["include/ignition/gui.hh"],
    # TODO: centralize this logic, as it is used here, in sdformat.BUILD, and
    # in fcl.BUILD
    cmd = "(" + (
        "echo '#include <ignition/gui/config.hh>' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g'"
    ) + ") > '$@'",
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources plus private headers.  The explicitly
# listed hdrs= matches upstream's public headers.
cc_library(
    name = "ignition_gui",
    srcs = [
        "include/ignition/gui/config.hh",  # from cmake_configure_file above
        "src/Iface.cc",
        "src/ign.cc",
        "src/MainWindow.cc",
        "src/Plugin.cc",
        "src/plugins/ImageDisplay.cc",
        "src/plugins/Publisher.cc",
        "src/plugins/Requester.cc",
        "src/plugins/Responder.cc",
        "src/plugins/TimePanel.cc",
        "src/plugins/TopicEcho.cc",
    ],
    hdrs = public_headers,
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ignition_common",
        "@ignition-transport3",
        "@Qt5Core",
    ],
)
