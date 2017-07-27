# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

# Generates config.hh based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/config.hh.in",
    out = "include/ignition/rendering/config.hh",
    cmakelists = ["CMakeLists.txt"],
    defines = [
        # It would be nice to get this information directly from CMakeLists.txt,
        # but it ends up being too hard.  ignition-gui sets a project name
        # as "ignition-gui<version>", and then uses CMake substring to pick
        # that version out.  We'd have to extend the cmake_configure_file
        # functionality to do the same, and I'm not sure it is worth it.  We
        # just hard code the major version here.
        "PROJECT_NAME=ignition-rendering",
        'IGN_RENDERING_PLUGIN_PATH="ign-rendering-0/plugins"',
    ],
)

public_headers = [
    "include/ignition/rendering/ArrowVisual.hh",
    "include/ignition/rendering/AxisVisual.hh",
    "include/ignition/rendering/Camera.hh",
    "include/ignition/rendering/CompositeVisual.hh",
    "include/ignition/rendering/Geometry.hh",
    "include/ignition/rendering/Image.hh",
    "include/ignition/rendering/Light.hh",
    "include/ignition/rendering/Material.hh",
    "include/ignition/rendering/MeshDescriptor.hh",
    "include/ignition/rendering/Mesh.hh",
    "include/ignition/rendering/Node.hh",
    "include/ignition/rendering/Object.hh",
    "include/ignition/rendering/PixelFormat.hh",
    "include/ignition/rendering/RenderEngine.hh",
    "include/ignition/rendering/RenderEngineManager.hh",
    "include/ignition/rendering/RenderingIface.hh",
    "include/ignition/rendering/RenderTypes.hh",
    "include/ignition/rendering/Scene.hh",
    "include/ignition/rendering/Sensor.hh",
    "include/ignition/rendering/ShaderType.hh",
    "include/ignition/rendering/Storage.hh",
    "include/ignition/rendering/SystemPaths.hh",
    "include/ignition/rendering/Util.hh",
    "include/ignition/rendering/Visual.hh",
]

public_base_headers = [
    "include/ignition/rendering/base/BaseArrowVisual.hh",
    "include/ignition/rendering/base/BaseAxisVisual.hh",
    "include/ignition/rendering/base/BaseCamera.hh",
    "include/ignition/rendering/base/BaseGeometry.hh",
    "include/ignition/rendering/base/BaseLight.hh",
    "include/ignition/rendering/base/BaseMaterial.hh",
    "include/ignition/rendering/base/BaseMesh.hh",
    "include/ignition/rendering/base/BaseNode.hh",
    "include/ignition/rendering/base/BaseObject.hh",
    "include/ignition/rendering/base/BaseRenderEngine.hh",
    "include/ignition/rendering/base/BaseRenderTarget.hh",
    "include/ignition/rendering/base/BaseRenderTypes.hh",
    "include/ignition/rendering/base/BaseScene.hh",
    "include/ignition/rendering/base/BaseSensor.hh",
    "include/ignition/rendering/base/BaseStorage.hh",
    "include/ignition/rendering/base/BaseVisual.hh",
]

public_ogre_headers = [
    "include/ignition/rendering/ogre/OgreArrowVisual.hh",
    "include/ignition/rendering/ogre/OgreAxisVisual.hh",
    "include/ignition/rendering/ogre/OgreCamera.hh",
    "include/ignition/rendering/ogre/OgreConversions.hh",
    "include/ignition/rendering/ogre/OgreGeometry.hh",
    "include/ignition/rendering/ogre/OgreIncludes.hh",
    "include/ignition/rendering/ogre/OgreLight.hh",
    "include/ignition/rendering/ogre/OgreMaterial.hh",
    "include/ignition/rendering/ogre/OgreMeshFactory.hh",
    "include/ignition/rendering/ogre/OgreMesh.hh",
    "include/ignition/rendering/ogre/OgreNode.hh",
    "include/ignition/rendering/ogre/OgreObject.hh",
    "include/ignition/rendering/ogre/OgreRenderEngine.hh",
    "include/ignition/rendering/ogre/OgreRenderTarget.hh",
    "include/ignition/rendering/ogre/OgreRenderTypes.hh",
    "include/ignition/rendering/ogre/OgreRTShaderSystem.hh",
    "include/ignition/rendering/ogre/OgreScene.hh",
    "include/ignition/rendering/ogre/OgreSensor.hh",
    "include/ignition/rendering/ogre/OgreStorage.hh",
    "include/ignition/rendering/ogre/OgreVisual.hh",
]

# Generates rendering.hh, which consists of #include statements for all of the
# public headers in the library.  The first line is
# '#include <ignition/rendering/config.hh>' followed by one line like
# '#include <ignition/rendering/ArrowVisual.hh>' for each non-generated header.
genrule(
    name = "renderinghh_genrule",
    srcs = public_headers,
    outs = ["include/ignition/rendering.hh"],
    # TODO: centralize this logic, as it is used here, in sdformat.BUILD, and
    # in fcl.BUILD
    cmd = "(" + (
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g'"
    ) + ") > '$@'",
)

genrule(
    name = "basehh_genrule",
    srcs = public_base_headers,
    outs = ["include/ignition/rendering/base/base.hh"],
    # TODO: centralize this logic, as it is used here, in sdformat.BUILD, and
    # in fcl.BUILD
    cmd = "(" + (
        "echo '#include <ignition/rendering/config.hh>' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g'"
    ) + ") > '$@'",
)

genrule(
    name = "ogrehh_genrule",
    srcs = public_ogre_headers,
    outs = ["include/ignition/rendering/ogre/ogre.hh"],
    # TODO: centralize this logic, as it is used here, in sdformat.BUILD, and
    # in fcl.BUILD
    cmd = "(" + (
        "echo '#include <ignition/rendering/config.hh>' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g'"
    ) + ") > '$@'",
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources plus private headers.  The explicitly
# listed hdrs= matches upstream's public headers.
cc_library(
    name = "ignition_rendering",
    srcs = [
        "include/ignition/rendering/config.hh",  # from cmake_configure_file above
        "include/ignition/rendering/base/base.hh",  # from cmake_configure_file above
        "include/ignition/rendering/ogre/ogre.hh",  # from cmake_configure_file above
        "src/base/BaseObject.cc",
        "src/base/BaseRenderEngine.cc",
        "src/base/BaseScene.cc",
        "src/Image.cc",
        "src/MeshDescriptor.cc",
        "src/ogre/OgreArrowVisual.cc",
        "src/ogre/OgreAxisVisual.cc",
        "src/ogre/OgreCamera.cc",
        "src/ogre/OgreConversions.cc",
        "src/ogre/OgreGeometry.cc",
        "src/ogre/OgreLight.cc",
        "src/ogre/OgreMaterial.cc",
        "src/ogre/OgreMesh.cc",
        "src/ogre/OgreMeshFactory.cc",
        "src/ogre/OgreNode.cc",
        "src/ogre/OgreObject.cc",
        "src/ogre/OgreRenderEngine.cc",
        "src/ogre/OgreRenderTarget.cc",
        "src/ogre/OgreRTShaderSystem.cc",
        "src/ogre/OgreScene.cc",
        "src/ogre/OgreSensor.cc",
        "src/ogre/OgreVisual.cc",
        "src/PixelFormat.cc",
        "src/RenderEngineManager.cc",
        "src/RenderingIface.cc",
        "src/ShaderType.cc",
        "src/SystemPaths.cc",
    ],
    defines = [
        # FIXME(clalancette): we should not hard-code these, particularly the second
        # one which can be found by running "pkg-config --variable-plugindir OGRE"
        'IGN_RENDERING_PLUGIN_PATH=\\"ign-rendering-0/plugins\\"',
        'OGRE_RESOURCE_PATH=\\"/usr/lib/x86_64-linux-gnu/OGRE-1.9.0\\"',
    ],
    hdrs = public_headers + public_base_headers + public_ogre_headers,
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ignition-common0",
        "@OGRE",
        "@OGRE-Paging",
    ],
)
