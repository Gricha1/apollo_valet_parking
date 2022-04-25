load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])


py_library(
    name = "update_config",
    srcs = ["update_config.py"],
    deps = [
    ],
)

py_library(
    name = "generate_trajectory",
    srcs = ["generate_trajectory.py"],
    deps = [
    ],
)

py_binary(
    name = "test_python",
    srcs = ["test_python.py"],
    deps = [
	"//cyber/python/cyber_py3:cyber",
    "//cyber/python/cyber_py3:cyber_time",
	"//modules/routing/proto:routing_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
    "//modules/planning/proto:planning_py_pb2",
    "//modules/tools/custom_2:validateModel",
    ],
)

py_library(
    name = "validateModel",
    srcs = ["validateModel.py"],
    deps = [
        "//modules/tools/custom_2/EnvLib:ObstGeomEnv",
        "//modules/tools/custom_2/planning:generateMap",
        "//modules/tools/custom_2/policy_gradient:utlis",
        "//modules/tools/custom_2:generate_trajectory",
        "//modules/tools/custom_2:update_config",
    ],
)

py_binary(
    name = "get_parking_points",
    srcs = ["get_parking_points.py"],
    deps = [
    ],
)


py_binary(
    name = "bridge_new_1",
    srcs = ["bridge_new_1.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/planning/proto:planning_py_pb2",
    ],
)
