load("@//tools:ign-gui.bzl", "IGN_GUI_PLUGIN_PATH")

cc_library(
    name = "_libRenderWidget.so",
    srcs = [
        "RenderWidget.cc",
        "@ignition_rendering//:libignition_rendering.so",
    ],
    deps = [
        # clalancette: it is *critical* that we only depend on the headers
        # from ignition-gui here.  If we use the shared_library dependency,
        # then a whole extra copy of the ignition-gui code gets linked into
        # the plugins.  When that plugin gets loaded, it ends up doing static
        # initialization of C++ global objects again (which ignition-gui uses),
        # blowing everything up.  In theory, the same situation applies for
        # ignition-common and ignition-rendering.  We fix ignition-common the
        # same way as ignition-gui here (by only depending on the headers), but
        # ignition-rendering is different for reasons I don't fully understand.
        "@ignition_common//:ignition_common_headers_only",
        "@ignition_gui//:ignition_gui_headers_only",
        "@ignition_rendering//:ignition_rendering_headers_only",
    ],
    linkstatic = 1,
)

cc_binary(
    name = "libRenderWidget.so",
    visibility = ["//visibility:public"],
    linkshared = 1,
    linkstatic = 1,
    deps = [
        ":_libRenderWidget.so",
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
    ],
)
