# -*- python -*-

load("@//tools:qt.bzl", "qt_moc_gen")

IGN_GUI_PLUGIN_PATH="ign-gui-0/plugins"

def ign_gui_create_plugins(plugin_list, headers):
    for plugin in plugin_list:
        filename = plugin + ".cc"
        srcfile = "src/plugins/" + filename
        incfile = "include/ignition/gui/plugins/" + plugin + ".hh"
        soname = "lib" + plugin + ".so"
        moc_filename = "moc_" + filename

        qt_moc_gen(
            name = "moc_" + plugin,
            hdr = incfile,
            out = moc_filename,
        )

        native.cc_library(
            name = IGN_GUI_PLUGIN_PATH + "/_" + soname,
            srcs = [
                srcfile,
                moc_filename,
                incfile,
            ] + headers,
            includes = ["include"],
            deps = [
                "@ignition_common//:ignition_common_shared_library",
                "@Qt5Core",
            ],
            linkstatic = 0,
        )

        native.cc_binary(
            name = IGN_GUI_PLUGIN_PATH + "/" + soname,
            visibility = ["//visibility:public"],
            linkshared = 1,
            linkstatic = 0,
            deps = [
                IGN_GUI_PLUGIN_PATH + "/_" + soname,
            ],
        )

def ign_gui_plugin_dep_name(name):
    return "@ignition_gui//:" + IGN_GUI_PLUGIN_PATH + "/lib" + name + ".so"
