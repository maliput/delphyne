load("@//tools:qt.bzl", "qt_moc_gen")

qt_moc_gen(
    name = "renderwidget_moc",
    hdr = "RenderWidget.hh",
    out = "moc_renderwidget.cc",
)

cc_library(
    name = "_libRenderWidget.so",
    srcs = [
        "OrbitViewControl.cc",
        "RenderWidget.cc",
        "moc_renderwidget.cc", # from qt_moc_gen above
    ],
    hdrs = [
        "OrbitViewControl.hh",
        "RenderWidget.hh",
    ],
    deps = [
        "@ignition-common0",
        "@ignition-gui",
        "@ignition-rendering",
        "@ignition-transport3",
    ],
    linkstatic = 0,
)

cc_binary(
    name = "libRenderWidget.so",
    linkshared = 1,
    linkstatic = 0,
    deps = [
        ":_libRenderWidget.so",
    ],
)

qt_moc_gen(
    name = "teleopwidget_moc",
    hdr = "TeleopWidget.hh",
    out = "moc_teleopwidget.cc",
)

cc_library(
    name = "_libTeleopWidget.so",
    srcs = [
        "TeleopWidget.cc",
        "moc_teleopwidget.cc", # from qt_moc_gen above
    ],
    hdrs = [
        "TeleopWidget.hh",
    ],
    deps = [
        "//bridge:ignition-custom-messages",
        "@ignition_common//:ignition_common_shared_library",
        "@ignition_gui//:ignition_gui_shared_library",
        "@ignition-transport3",
    ],
    linkstatic = 0,
    copts = ["-Ibridge"],  # TODO(clalancette): The dependency from ignition-custom-messages isn't working properly for some reason
)

cc_binary(
    name = "libTeleopWidget.so",
    linkshared = 1,
    linkstatic = 0,
    deps = [
        ":_libTeleopWidget.so",
    ],
)

cc_binary(
    name = "visualizer",
    visibility = [
        "//visibility:public"
    ],
    srcs = [
        "visualizer.cc",
    ],
    deps = [
        "@ignition-common0",
        "@ignition-gui",
    ],
    linkstatic = 0,
    copts = [
        # clalancette: The visualizer binary needs to be able to find the
        # libRenderWidget.so library at runtime so it can dlopen() it.
        # Both the visualizer binary and the libRenderWidget.so library
        # end up in the same sub-directory, but that directory is at
        # different directory structures depending on where you are running
        # the binary from (this is because of Bazel's penchant for symlinking
        # things all over the place).  When running visualizer with
        # "bazel run //visualizer:visualizer", for instance, the PWD is
        # some crazy path down in bazel-out.  Thus, the only reliable way
        # to always find libRenderWidget.so is with this hard-coded
        # directory, which is always one-level down from the PWD when
        # the binary is launched.
        "-DPLUGIN_INSTALL_PATH=\\\"visualizer\\\"",
    ],
    data = [
        ":libRenderWidget.so",
        ":libTeleopWidget.so",
        ":layout.config",
    ],
)
