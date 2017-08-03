# -*- python -*-

def ign_gui_create_plugin(name, srcfile):
    native.cc_library(
        name = "ign-gui-0/plugins/_" + name,
        srcs = [
            srcfile,
            "@ignition_common//:libignition_common.so",
        ],
        includes = ["include"],
        deps = [
            "@ignition_common//:ignition_common_shared_library",
            "@Qt5Core",
        ],
        linkstatic = 1,
    )

    native.cc_binary(
        name = "ign-gui-0/plugins/" + name,
        linkshared = 1,
        linkstatic = 1,
        deps = [
            ":ign-gui-0/plugins/_" + name,
        ],
    )

def ign_gui_plugin_dep_name(name):
    return ":ign-gui-0/plugins/" + name

IGN_GUI_PLUGIN_PATH="ign-gui-0/plugins"
