load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
)

cc_library(
    name = "ignition-custom-messages",
    srcs = ["protobuf/headers/automotive_driving_command.pb.cc",
    ],
    hdrs = ["protobuf/headers/automotive_driving_command.pb.h",
    ],
    deps = [
        "@ignition-transport3",
    ],
)

cc_library(
    name = "lcm-to-ign-translation",
    srcs = [
        "lcm_to_ign_translation.cc",
    ],
    hdrs = [
        "lcm_to_ign_translation.hh",
        "translate_exception.hh",
    ],
    deps = [
        "@ignition-transport3",
        "@lcm",
        "//bridge:drake-lcmt-headers",
    ],
)

cc_library(
    name = "ign-to-lcm-translation",
    srcs = [
        "ign_to_lcm_translation.cc",
    ],
    hdrs = [
        "ign_to_lcm_translation.hh",
        "translate_exception.hh",
        "abstract_repeater.hh",
    ],
    deps = [
        "@ignition-transport3",
        "@lcm",
        "//bridge:ignition-custom-messages",
        "//bridge:drake-lcmt-headers",
    ],
)

cc_library(
    name = "service-to-channel-translation",
    srcs = ["service_to_channel_translation.cc"],
    hdrs = ["service_to_channel_translation.hh"],
    deps = [
        "@ignition-transport3",
        "@lcm",
        "//bridge:drake-lcmt-headers",
    ],
)

cc_library(
    name = "lcm-channel-repeater",
    hdrs = ["lcm_channel_repeater.hh",
    ],
    deps = [
        "@ignition-transport3",
        "@lcm",
        "//bridge:drake-lcmt-headers",
        "@ignition-common0",
    ],
)

cc_library(
    name = "repeater-manager",
    hdrs = [
        "repeater_manager.hh",
        "abstract_repeater.hh",
    ],
    srcs = ["repeater_manager.cc",
    ],
    deps = [
        "@lcm",
        "@ignition-common0",
        "@ignition-transport3",
        "//bridge:repeater-factory",
    ],
)

cc_library(
    name = "ign-topic-repeater",
    hdrs = ["ign_topic_repeater.hh",
    ],
    deps = [
        "@ignition-transport3",
        "@lcm",
        "//bridge:drake-lcmt-headers",
        "//bridge:ign-to-lcm-translation",
        "@ignition-common0",
    ],
)

cc_library(
    name = "ign-service-converter",
    hdrs = ["ign_service_converter.hh"],
    deps = [
        "@lcm",
        "//bridge:service-to-channel-translation",
    ],
)

cc_library(
    name = "drake-lcmt-headers",
    hdrs = ["drake/lcmt_viewer_command.hpp",
            "drake/lcmt_viewer_draw.hpp",
            "drake/lcmt_viewer_geometry_data.hpp",
            "drake/lcmt_viewer_link_data.hpp",
            "drake/lcmt_viewer_load_robot.hpp",
            "drake/lcmt_driving_command_t.hpp",
    ],
)

cc_library(
    name = "repeater-factory",
    srcs = [
        "repeater_factory.cc",
    ],
    hdrs = ["repeater_factory.hh",
    ],
    deps = [
        "//bridge:ign-topic-repeater",
    ],
)

cc_binary(
    name = "lcm-ign-transport-bridge",
    srcs = ["lcm_ign_transport_bridge.cc",
    ],
    deps = [
        "@ignition-transport3",
        "@lcm",
        "@ignition-common0",
        "//bridge:lcm-to-ign-translation",
        "//bridge:ign-to-lcm-translation",
        "//bridge:drake-lcmt-headers",
        "//bridge:lcm-channel-repeater",
        "//bridge:ign-service-converter",
        "//bridge:ign-topic-repeater",
        "//bridge:repeater-factory",
        "//bridge:repeater-manager",
    ],
)

cc_binary(
    name = "lcm-mock-robot-publisher",
    srcs = [
        "lcm_mock_robot_publisher.cc",
    ],
    deps = [
        "@ignition-common0",
        "@lcm",
        "//bridge:drake-lcmt-headers",
        "//bridge:ignition-custom-messages",
    ],
)

py_library(
    name = "launcher",
    srcs = ["launcher.py"],
)

py_binary(
    name = "mocked-robot-demo",
    srcs = ["mocked_robot_demo.py"],
    data = [
        ":lcm-mock-robot-publisher",
        ":lcm-ign-transport-bridge",
        "//visualizer:visualizer",
    ],
    deps = ["//bridge:launcher"],
    main = "mocked_robot_demo.py",
    args = [
        "$(location :lcm-mock-robot-publisher)",
        "$(location :lcm-ign-transport-bridge)",
        "$(location //visualizer:visualizer)",
    ],
)

py_binary(
    name = "demo-launcher",
    srcs = ["demo_launcher.py"],
    deps = [
        "//bridge:launcher",
        "@lcm//:lcm-python",
    ],
    data = [
        ":lcm-ign-transport-bridge",
        "//visualizer:visualizer",
    ],
    main = "demo_launcher.py",
)

drake_cc_googletest(
    name = "lcm_to_ign_translation_test",
    deps = [
        "@lcm",
        "//bridge:lcm-to-ign-translation",
        "//bridge:drake-lcmt-headers",
    ],
)

drake_cc_googletest(
    name = "ign_to_lcm_translation_test",
    deps = [
        "@lcm",
        "//bridge:ign-to-lcm-translation",
        "//bridge:drake-lcmt-headers",
        "//bridge:ignition-custom-messages",
    ],
)

drake_cc_googletest(
    name = "lcm_channel_repeater_test",
    deps = [
        "@lcm",
        "//bridge:lcm-to-ign-translation",
        "//bridge:lcm-channel-repeater",
        "//bridge:drake-lcmt-headers",
    ],
)

drake_cc_googletest(
    name = "ign_service_converter_test",
    deps = [
        "@lcm",
        "//bridge:drake-lcmt-headers",
        "//bridge:ign-service-converter",
    ],
)

drake_cc_googletest(
    name = "service_to_channel_translation_test",
    deps = [
        "@lcm",
        "//bridge:drake-lcmt-headers",
        "//bridge:ign-service-converter",
    ],
)

drake_cc_googletest(
    name = "repeater_factory_test",
    deps = [
        "@lcm",
        "@ignition-transport3",
        "//bridge:repeater-factory",
    ],
)

drake_cc_googletest(
    name = "repeater_manager_test",
    deps = [
        "@lcm",
        "@ignition-transport3",
        "//bridge:repeater-factory",
        "//bridge:repeater-manager",
    ],
)
