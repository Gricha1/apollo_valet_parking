load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])


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
