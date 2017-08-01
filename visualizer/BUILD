cc_library(
    name = "_libGuiRenderWidget.so",
    srcs = [
        "GuiRenderWidget.cc",
        "@ignition_common//:libignition_common.so",
        "@ignition_gui//:libignition_gui.so",
        "@ignition_rendering//:libignition_rendering.so",
    ],
    hdrs = [
        "GuiRenderWidget.hh"
    ],
    deps = [
        "@ignition_common//:ignition_common_shared_library",
        "@ignition_gui//:ignition_gui_shared_library",
        "@ignition_rendering//:ignition_rendering_shared_library",
    ],
    linkstatic = 1,
)

cc_binary(
    name = "libGuiRenderWidget.so",
    visibility = ["//visibility:public"],
    linkshared = 1,
    linkstatic = 1,
    deps = [
        ":_libGuiRenderWidget.so",
    ],
)

cc_binary(
    name = "visualizer",
    srcs = [
        "visualizer.cc",
        "@ignition_gui//:libignition_gui.so",
    ],
    deps = [
        "@ignition_gui//:ignition_gui_shared_library",
    ],
    copts = [
        # clalancette: The visualizer binary needs to be able to find the
        # libGuiRenderWidget.so library at runtime so it can dlopen() it.
        # Both the visualizer binary and the libGuiRenderWidget.so library
        # end up in the same sub-directory, but that directory is at
        # different directory structures depending on where you are running
        # the binary from (this is because of Bazel's penchant for symlinking
        # things all over the place).  When running visualizer with
        # "bazel run //visualizer:visualizer", for instance, the PWD is
        # some crazy path down in bazel-out.  Thus, the only reliable way
        # to always find libGuiRenderWidget.so is with this hard-coded
        # directory, which is always one-level down from the PWD when
        # the binary is launched.
        '-DPLUGIN_INSTALL_PATH=\\"visualizer\\"',
    ],
    data = [
        ":libGuiRenderWidget.so",
    ],
)
