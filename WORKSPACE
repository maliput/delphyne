workspace(name = "delphyne")

load("//tools:github.bzl", "github_archive")
load("//tools:bitbucket.bzl", "bitbucket_archive")

local_repository(
    name = "kythe",
    path = "tools/third_party/kythe",
)

load("@kythe//tools/build_rules/config:pkg_config.bzl", "pkg_config_package")

pkg_config_package(
    name = "avcodec",
    modname = "libavcodec",
)

pkg_config_package(
    name = "avdevice",
    modname = "libavdevice",
)

pkg_config_package(
    name = "avformat",
    modname = "libavformat",
)

pkg_config_package(
    name = "avutil",
    modname = "libavutil",
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
    name = "ignition-math3",
    modname = "ignition-math3",
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

load("//tools:python.bzl", "python_repository")

python_repository(
    name = "python",
    version = "2.7",
)

bitbucket_archive(
    name = "ignition_common",
    repository = "ignitionrobotics/ign-common",
    commit = "b54a1ff",
    sha256 = "4c7ec0fbf41f5465adc4d5163436263e3295cb07274b70bf7f974572e17b88bd",
    strip_prefix = "ignitionrobotics-ign-common-b54a1ff745fa",
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
    commit = "4323a26",
    sha256 = "de7426baf61a3c2fa584de7b769c0a25f446d8a81ca2433b9a8df1a783174235",
    strip_prefix = "ignitionrobotics-ign-rendering-4323a267a516",
    build_file = "tools/ignition_rendering.BUILD",
)

github_archive(
    name = "lcm",
    repository = "lcm-proj/lcm",
    commit = "c0a0093a950fc83e12e8d5918a0319b590356e7e",
    sha256 = "f967e74e639ea56318242e93c77a15a504345c8200791cab70d9dad86aa969b2",
    build_file = "tools/lcm.BUILD",
)

github_archive(
    name = "gtest",
    repository = "google/googletest",
    commit = "release-1.8.0",
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",  # noqa
    build_file = "tools/gtest.BUILD",
)

