# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

# Generates config.hh based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/config.hh.in",
    out = "include/ignition/common/config.hh",
    cmakelists = ["CMakeLists.txt"],
    defines = [
        # It would be nice to get this information directly from CMakeLists.txt,
        # but it ends up being too hard.  ignition-common sets a project name
        # as "ignition-common<version>", and then uses CMake substring to pick
        # that version out.  We'd have to extend the cmake_configure_file
        # functionality to do the same, and I'm not sure it is worth it.  We
        # just hard code the major version here.
        "PROJECT_NAME=ignition-common0",
        "PROJECT_NAME_NO_VERSION=ignition-common",
        "PROJECT_MAJOR_VERSION=0",
        "PROJECT_VERSION_FULL=0.4.0",
    ],
)

public_headers = [
    "include/ignition/common/Animation.hh",
    "include/ignition/common/AudioDecoder.hh",
    "include/ignition/common/Base64.hh",
    "include/ignition/common/Battery.hh",
    "include/ignition/common/BVHLoader.hh",
    "include/ignition/common/ColladaExporter.hh",
    "include/ignition/common/ColladaLoader.hh",
    "include/ignition/common/config.hh",
    "include/ignition/common/Console.hh",
    "include/ignition/common/Dem.hh",
    "include/ignition/common/EnumIface.hh",
    "include/ignition/common/Event.hh",
    "include/ignition/common/ffmpeg_inc.hh",
    "include/ignition/common/GTSMeshUtils.hh",
    "include/ignition/common/HeightmapData.hh",
    "include/ignition/common/ImageHeightmap.hh",
    "include/ignition/common/Image.hh",
    "include/ignition/common/KeyEvent.hh",
    "include/ignition/common/KeyFrame.hh",
    "include/ignition/common/MaterialDensity.hh",
    "include/ignition/common/Material.hh",
    "include/ignition/common/MeshCSG.hh",
    "include/ignition/common/MeshExporter.hh",
    "include/ignition/common/Mesh.hh",
    "include/ignition/common/MeshLoader.hh",
    "include/ignition/common/MeshManager.hh",
    "include/ignition/common/MouseEvent.hh",
    "include/ignition/common/MovingWindowFilter.hh",
    "include/ignition/common/NodeAnimation.hh",
    "include/ignition/common/NodeTransform.hh",
    "include/ignition/common/OBJLoader.hh",
    "include/ignition/common/PluginInfo.hh",
    "include/ignition/common/PluginLoader.hh",
    "include/ignition/common/PluginMacros.hh",
    "include/ignition/common/SemanticVersion.hh",
    "include/ignition/common/SingletonT.hh",
    "include/ignition/common/SkeletonAnimation.hh",
    "include/ignition/common/Skeleton.hh",
    "include/ignition/common/SkeletonNode.hh",
    "include/ignition/common/STLLoader.hh",
    "include/ignition/common/StringUtils.hh",
    "include/ignition/common/SubMesh.hh",
    "include/ignition/common/SVGLoader.hh",
    "include/ignition/common/System.hh",
    "include/ignition/common/SystemPaths.hh",
    "include/ignition/common/Time.hh",
    "include/ignition/common/Timer.hh",
    "include/ignition/common/Types.hh",
    "include/ignition/common/URI.hh",
    "include/ignition/common/Util.hh",
    "include/ignition/common/Uuid.hh",
    "include/ignition/common/VideoEncoder.hh",
    "include/ignition/common/Video.hh",
    "include/ignition/common/WorkerPool.hh",
]

# Generates common.hh, which consists of #include statements for all of the
# public headers in the library.  The first line is
# '#include <ignition/common/config.hh>' followed by one line like
# '#include <ignition/common/Example.hh>' for each non-generated header.
genrule(
    name = "commonhh_genrule",
    srcs = public_headers,
    outs = ["include/ignition/common.hh"],
    # TODO: centralize this logic, as it is used here, in sdformat.BUILD, and
    # in fcl.BUILD
    cmd = "(" + (
        "echo '#include <ignition/common/config.hh>' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g'"
    ) + ") > '$@'",
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources plus private headers.  The explicitly
# listed hdrs= matches upstream's public headers.
cc_library(
    name = "ignition_common",
    srcs = [
        "include/ignition/common/config.hh",  # from cmake_configure_file above
        "src/Animation.cc",
        "src/AudioDecoder.cc",
        "src/Base64.cc",
        "src/Battery.cc",
        "src/BVHLoader.cc",
        "src/ColladaExporter.cc",
        "src/ColladaLoader.cc",
        "src/Console.cc",
        "src/Dem.cc",
        "src/Event.cc",
        "src/ffmpeg_inc.cc",
        "src/GTSMeshUtils.cc",
        "src/Image.cc",
        "src/ImageHeightmap.cc",
        "src/KeyEvent.cc",
        "src/KeyFrame.cc",
        "src/Material.cc",
        "src/MaterialDensity.cc",
        "src/Mesh.cc",
        "src/MeshCSG.cc",
        "src/MeshExporter.cc",
        "src/MeshManager.cc",
        "src/MouseEvent.cc",
        "src/NodeAnimation.cc",
        "src/NodeTransform.cc",
        "src/OBJLoader.cc",
        "src/PluginLoader.cc",
        "src/SemanticVersion.cc",
        "src/SkeletonAnimation.cc",
        "src/Skeleton.cc",
        "src/SkeletonNode.cc",
        "src/STLLoader.cc",
        "src/StringUtils.cc",
        "src/SubMesh.cc",
        "src/SVGLoader.cc",
        "src/SystemPaths.cc",
        "src/Time.cc",
        "src/Timer.cc",
        "src/tiny_obj_loader.h",
        "src/URI.cc",
        "src/Util.cc",
        "src/Uuid.cc",
        "src/Video.cc",
        "src/VideoEncoder.cc",
        "src/WorkerPool.cc",
    ],
    hdrs = public_headers,
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ignition-transport3",
        "@glib",
        "@uuid",
    ],
    linkopts = [
    ],
)
