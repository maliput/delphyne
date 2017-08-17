load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
)

cc_library(
    name = "lcm-to-ign-translation",
    srcs = ["lcm_to_ign_translation.cc",],
    hdrs = ["lcm_to_ign_translation.hh",
    ],
    deps = [
        "@ignition-transport3",
        "@lcm",
        "//bridge:drake-lcmt-headers"
    ],
)

cc_library(
    name = "drake-lcmt-headers",
    hdrs = ["drake/lcmt_viewer_command.hpp",
            "drake/lcmt_viewer_draw.hpp",
            "drake/lcmt_viewer_geometry_data.hpp",
            "drake/lcmt_viewer_link_data.hpp",
            "drake/lcmt_viewer_load_robot.hpp"
    ],
)

cc_binary(
    name = "lcm-to-ign-transport-bridge",
    srcs = [
        "lcm_to_ign_transport_bridge.cc",
        "lcm_channel_repeater.hh",
    ],
    deps = [
        "@ignition-transport3",
        "@lcm",
        "@ignition_common//:ignition_common_shared_library",
        "//bridge:lcm-to-ign-translation",
        "//bridge:drake-lcmt-headers"
    ],
    data = [
        "drake/car/car.vtp",
        "drake/package.xml",
        "drake/car/package.xml"
    ],
)

cc_binary(
    name = "lcm-mock-robot-publisher",
    srcs = [
        "lcm_mock_robot_publisher.cc",
        "drake/lcmt_viewer_geometry_data.hpp",
        "drake/lcmt_viewer_link_data.hpp",
        "drake/lcmt_viewer_load_robot.hpp",
    ],
    deps = [
        "@ignition_common//:ignition_common_shared_library",
        "@lcm",
    ],
)

py_binary(
    name = "mocked-robot-demo",
    srcs = ["mocked_robot_demo.py"],
    data = [
        ":lcm-mock-robot-publisher",
        ":lcm-to-ign-transport-bridge"
    ],
    main = "mocked_robot_demo.py",
    args = [
        "$(location :lcm-mock-robot-publisher)",
        "$(location :lcm-to-ign-transport-bridge)"
    ],
)

drake_cc_googletest(
    name = "lcm_to_ign_translation_test",
    deps = [
        "@lcm",
        "//bridge:lcm-to-ign-translation",
        "//bridge:drake-lcmt-headers"
    ],
)
