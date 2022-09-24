load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "valet_parking_request",
    srcs = ["valet_parking_request.py"],
    deps = [
	"//cyber/python/cyber_py3:cyber",
    "//cyber/python/cyber_py3:cyber_time",
	"//modules/routing/proto:routing_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
    "//modules/planning/proto:planning_py_pb2"
    ],
)

py_binary(
    name = "valet_parking_request_test",
    srcs = ["valet_parking_request_test.py"],
    deps = [
	"//cyber/python/cyber_py3:cyber",
    "//cyber/python/cyber_py3:cyber_time",
	"//modules/routing/proto:routing_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2"
    ],
)

py_binary(
    name = "create_svl_scene",
    srcs = ["create_svl_scene.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/planning/proto:planning_py_pb2",
    ],
)

py_binary(
    name = "create_dynamic_obsts_publisher",
    srcs = ["create_dynamic_obsts_publisher.py"],
    deps = [
	"//cyber/python/cyber_py3:cyber",
    "//cyber/python/cyber_py3:cyber_time",
	"//modules/routing/proto:routing_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
    "//modules/planning/proto:planning_py_pb2",
    "//modules/localization/proto:localization_py_pb2"
    ],
)