load("@//tools:ign-gui.bzl", "ign_gui_plugin_dep_name", "IGN_GUI_PLUGIN_PATH")
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
        "moc_renderwidget.cc", # from qt_moc_get above
    ],
    hdrs = [
        "OrbitViewControl.hh",
        "RenderWidget.hh",
    ],
    deps = [
        "@ignition_common//:ignition_common_shared_library",
        "@ignition_gui//:ignition_gui_shared_library",
        "@ignition_rendering//:ignition_rendering_shared_library",
        "@ignition-transport3",
    ],
    linkstatic = 0,
)

cc_binary(
    name = "libRenderWidget.so",
    visibility = ["//visibility:public"],
    linkshared = 1,
    linkstatic = 0,
    deps = [
        ":_libRenderWidget.so",
    ],
)

cc_binary(
    name = "visualizer",
    visibility = ["//visibility:public"],
    srcs = [
        "visualizer.cc",
    ],
    deps = [
        "@ignition_gui//:ignition_gui_shared_library",
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
        '-DPLUGIN_INSTALL_PATH=\\"visualizer\\"',
        '-DIGN_GUI_DEFAULT_PLUGIN_PATH=\\"external/ignition_gui/' + IGN_GUI_PLUGIN_PATH + '\\"',
    ],
    data = [
        ":libRenderWidget.so",
        ign_gui_plugin_dep_name("TimePanel"),
        ign_gui_plugin_dep_name("TopicEcho"),
    ],
)
