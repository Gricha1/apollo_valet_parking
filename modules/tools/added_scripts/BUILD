load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])



py_binary(
    name = "publishing_obstacles-test-scene",
    srcs = ["publishing_obstacles-test-scene.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/routing/proto:routing_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
    ],
)

py_binary(
    name = "publishing_obstacles_many_obstacles_same_speed",
    srcs = ["publishing_obstacles_many_obstacles_same_speed.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/routing/proto:routing_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
    ],
)


py_binary(
    name = "sending_routing_request",
    srcs = ["sending_routing_request.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/routing/proto:routing_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
    ],
)

py_binary(
    name = "publishing_obstacles",
    srcs = ["publishing_obstacles.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/routing/proto:routing_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
    ],
)

