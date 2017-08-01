workspace(name = "drakaina")

load("//tools:github.bzl", "github_archive")
load("//tools:bitbucket.bzl", "bitbucket_archive")

local_repository(
    name = "kythe",
    path = "tools/third_party/kythe",
)

load("@kythe//tools/build_rules/config:pkg_config.bzl", "pkg_config_package")

pkg_config_package(
    name = "avdevice",
    modname = "libavdevice",
)

pkg_config_package(
    name = "gts",
    modname = "gts",
)

pkg_config_package(
    name = "glib",
    modname = "glib-2.0",
)

pkg_config_package(
    name = "ignition-transport3",
    modname = "ignition-transport3",
)

pkg_config_package(
    name = "OGRE",
    modname = "OGRE",
    additional_flag_defines = {"OGRE_RESOURCE_PATH": "--variable=plugindir"},
)

pkg_config_package(
    name = "OGRE-Paging",
    modname = "OGRE-Paging",
)

pkg_config_package(
    name = "OGRE-RTShaderSystem",
    modname = "OGRE-RTShaderSystem",
)

pkg_config_package(
    name = "Qt5Core",
    modname = "Qt5Core",
)

pkg_config_package(
    name = "Qt5Gui",
    modname = "Qt5Gui",
)

pkg_config_package(
    name = "Qt5Widgets",
    modname = "Qt5Widgets",
)

pkg_config_package(
    name = "tinyxml2",
    modname = "tinyxml2",
)

pkg_config_package(
    name = "uuid",
    modname = "uuid",
)

bitbucket_archive(
    name = "ignition_common",
    repository = "ignitionrobotics/ign-common",
    commit = "67a53f0d1de6",
    sha256 = "356d9def73d6fe7105fd3a7cbd25a6f66edcaaf47a8d71fccd96c0860999fd28",
    strip_prefix = "ignitionrobotics-ign-common-67a53f0d1de6",
    build_file = "tools/ignition_common.BUILD",
)

bitbucket_archive(
    name = "ignition_gui",
    repository = "ignitionrobotics/ign-gui",
    commit = "4400e7d9210e",
    sha256 = "eeac30aee85be4214ccb80aaec926c7bf1d7b08f5cbd58ad4da91985a180e4a6",
    strip_prefix = "ignitionrobotics-ign-gui-4400e7d9210e",
    build_file = "tools/ignition_gui.BUILD",
)

bitbucket_archive(
    name = "ignition_rendering",
    repository = "ignitionrobotics/ign-rendering",
    commit = "0ead5eb3dc20",
    sha256 = "43cfdbdef50382ea8a0b820b10320083f2c1257722ec16b7e57949468ba51fd8",
    strip_prefix = "ignitionrobotics-ign-rendering-0ead5eb3dc20",
    build_file = "tools/ignition_rendering.BUILD",
)

github_archive(
    name = "lcm",
    repository = "lcm-proj/lcm",
    commit = "c0a0093a950fc83e12e8d5918a0319b590356e7e",
    sha256 = "d5bb1a0153b9c1526590e7d65be8ca79e4f5e9bf4ce58178c992eaca49d17fb0",
    build_file = "tools/lcm.BUILD",
)
